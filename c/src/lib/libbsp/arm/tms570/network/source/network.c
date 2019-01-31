/*
 * RTEMS driver for M68360 SCC1 Ethernet
 *
 * W. Eric Norum
 * Saskatchewan Accelerator Laboratory
 * University of Saskatchewan
 * Saskatoon, Saskatchewan, CANADA
 * eric@skatter.usask.ca
 */
#include <bsp.h>
#include <bsp/tms570.h>
#include <stdio.h>
#include <errno.h>
#include <rtems/error.h>
#include <rtems/rtems_bsdnet.h>
#include <rtems/irq-extension.h>

#include <sys/param.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <bsp/irq.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>
#include "emac.h"

/*
 * Number of SCCs supported by this driver
 */

//#define TMS_INTC0_TX_VECTOR 0xFFF821E0
//#define TMS_INTC0_RX_VECTOR 0xFFF821E8
static uint8_t pbuf_array[MAX_RX_PBUF_ALLOC][MAX_TRANSFER_UNIT];
//static uint8_t rxDataBuf[MAX_RX_PBUF_ALLOC][MAX_TRANSFER_UNIT];
#define NTMSDRIVER	1
#define CHANNEL		0
uint32 EMACSwizzleData(uint32 word);
static void tms570_eth_init_buffer_descriptors(hdkif_t *hdkif);
boolean tms570_eth_send_raw(hdkif_t *hdkif, struct mbuf *mbuf);
u32_t sys_arch_sem_wait(rtems_id rtid, u32_t timeout);
void static tms570_eth_process_irq_request(void *arg);
tms570_eth_process_irq(void *arg);
static void tms570_eth_rx_pbuf_refill( struct tms_softc *sc, int single_fl);
static void tms570_eth_process_irq_rx(void *arg);
static void tms570_eth_process_irq_tx(void *arg);
tms570_eth_hw_set_TX_HDP(hdkif_t *hdkif, volatile struct emac_tx_bd *new_head);
tms570_eth_hw_set_RX_HDP(hdkif_t *hdkif, volatile struct emac_rx_bd *new_head);

/*
 * Default number of buffer descriptors set aside for this driver.
 * The number of transmit buffer descriptors has to be quite large
 * since a single frame often uses four or more buffer descriptors.
 */
#define RX_BUF_COUNT     15
#define TX_BUF_COUNT     4
#define TX_BD_PER_BUF    4
#define MIN_FRAME_LENGTH 64
#define MAX_FRAME_LENGTH 1500
#define pinMuxBaseReg ((long unsigned int) 0xFFFF1D10U) 
#define RegEdit ((long unsigned int) 0xFFFFEA38U)
#define SET_DRIVER_RMII 1
/*
 * RTEMS event used by interrupt handler to signal driver tasks.
 * This must not be any of the events used by the network task synchronization.
 */
#define INTERRUPT_EVENT	RTEMS_EVENT_1

/*
 * RTEMS event used to start transmit daemon.
 * This must not be the same as INTERRUPT_EVENT.
 */
#define START_TRANSMIT_EVENT	RTEMS_EVENT_2
/*
 * Receive buffer size -- Allow for a full ethernet packet including CRC
 */
#define RBUF_SIZE	1520

#if (MCLBYTES < RBUF_SIZE)
 error("Driver must have MCLBYTES > RBUF_SIZE");
#endif




/*
 * Per-device data
 */
struct tms_softc {
	struct arpcom	arpcom;
        rtems_id        rxDaemonTid;
        rtems_id        txDaemonTid;
	struct mbuf	**rxMbuf;
        uint32		rxBdCount;
        uint32		txBdCount;
        uint32             rxInterrupts;
        uint32            txInterrupts;
        hdkif_t         *hdkif;
};



static struct tms_softc tms[NTMSDRIVER];
hdkif_t hdkif_data[1];
//extern void *_RomBase;	/* From linkcmds */

uint32 EMACSwizzleData(uint32 word) {
		return
			(((word << 24U) & 0xFF000000U) |
	 		((word <<  8U) & 0x00FF0000U)  |
			((word >>  8U) & 0x0000FF00U)  |
			((word >> 24U) & 0x000000FFU));
}

#ifdef SENDULAN
static void tms570_eth_init_buffer_descriptors(hdkif_t *hdkif)
  
{
  uint32_t num_bd;
  volatile struct emac_tx_bd *curr_txbd;
  volatile struct emac_rx_bd *curr_rxbd;

  struct rxch *rxch;
  struct txch *txch;

  rxch = &(hdkif->rxch);
  txch = &(hdkif->txch);

//  rxch = &(nf_state->rxch);
//  txch = &(nf_state->txch);


  /*
   * Use the CPPI RAM to store RX/TX Buffer Descriptors (= BD).
   * 1/2 of CPPI RAM is used for TX BDs, another 1/2 for RX BDs.
   * All the TX BDs are 'owned' by the software. They are initialized
   * as a linked-list forming a ring. They are awaiting the application
   * to append pbuf payload to the them and correctly configure for
   * actual transmission.
   * Only such number of RX BDs is configured that the pbufs can be
   * allocated for (i.e. MAX_RX_PBUF_ALLOC). Pbufs are allocated from
   * the PBUF_POOL (thus the allocated pbufs might be chained).
   * Each payload part of a payload is them used to initialize single
   * RX BD. The RX BDs are then configured to be 'owned' bythe EMAC
   */

  /*
   * Initialize the Descriptor Memory For TX and RX
   * Only Channel 0 is supported for both TX and RX
   */
  txch->inactive_head = (volatile struct emac_tx_bd *)hdkif->emac_ctrl_ram;
  txch->inactive_tail = NULL;
  txch->active_head = NULL;
  txch->active_tail = NULL;

  /* Set the number of descriptors for the channel */
  /* take half of CPPI ram for TX BDs */

  num_bd = ((SIZE_EMAC_CTRL_RAM >> 1) / sizeof(struct emac_tx_bd))-1;
#if TMS570_NETIF_DEBUG
  tms570_eth_debug_printf("pocet bd %d\n", num_bd);
#endif

  curr_txbd = txch->inactive_head;

  /* Initialize all the TX buffer Descriptors */
  while (num_bd > 0) {
    curr_txbd->next = curr_txbd + 1;
    curr_txbd->flags_pktlen = 0;
    txch->inactive_tail = curr_txbd;
    curr_txbd = curr_txbd->next;
    num_bd--;
  }
  curr_txbd->next = NULL;

  /* Initialize the descriptors for the RX channel */
  rxch->inactive_head = ((volatile struct emac_rx_bd *)curr_txbd)+2;
//  rxch->freed_pbuf_len = MAX_RX_PBUF_ALLOC*PBUF_LEN_MAX;
  rxch->inactive_head->flags_pktlen = EMAC_BUF_DESC_OWNER;
  rxch->inactive_head->next = rxch->inactive_head + 1;
  curr_rxbd = rxch->inactive_head;

  num_bd = ((SIZE_EMAC_CTRL_RAM >> 1) / sizeof(struct emac_rx_bd))-1;

  while (num_bd > 0) {
    curr_rxbd = curr_rxbd->next;
    curr_rxbd->next = curr_rxbd + 1;
    curr_rxbd->flags_pktlen = EMAC_BUF_DESC_OWNER;
    curr_rxbd->mbuf = NULL;
    num_bd--;
  }
  curr_rxbd->next = NULL;
  rxch->inactive_tail = curr_rxbd;
  rxch->active_tail = NULL;
  rxch->active_head = NULL;
#if TMS570_NETIF_DEBUG
  tms570_eth_debug_show_rx(nf_state);
  tms570_eth_debug_show_tx(nf_state);
#endif

}
#else
void EMACDMAInit(hdkif_t *hdkif)
{

      uint32 num_bd, pbuf_cnt = 0U;
      volatile emac_tx_bd_t *curr_txbd, *last_txbd;
      txch *txch_dma;
      volatile emac_rx_bd_t *curr_bd, *last_bd;

      rxch_t *rxch_dma;
      uint8_t *p;

      txch_dma = &(hdkif->txchptr);

      /**
      * Initialize the Descriptor Memory For TX and RX
      * Only single channel is supported for both TX and RX
      */
      /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
      txch_dma->free_head = (volatile emac_tx_bd_t*)(hdkif->emac_ctrl_ram);
      txch_dma->next_bd_to_process = txch_dma->free_head;
      txch_dma->active_tail = NULL;

      /* Set the number of descriptors for the channel */
      num_bd = (SIZE_EMAC_CTRL_RAM >> 1U) / sizeof(emac_tx_bd_t);

      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
      curr_txbd = txch_dma->free_head;
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
      last_txbd = curr_txbd;

      /* Initialize all the TX buffer Descriptors */
      while(num_bd != 0U) {
        /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Struct pointer used for linked list is incremented." */
        curr_txbd->next = curr_txbd + 1U;
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
        curr_txbd->flags_pktlen = 0U;
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */   
        last_txbd = curr_txbd;
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */   
        curr_txbd = curr_txbd->next;
        num_bd--;
      }
      /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
      last_txbd->next = txch_dma->free_head;

      /* Initialize the descriptors for the RX channel */
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
      rxch_dma = &(hdkif->rxchptr);
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
      /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Struct pointer used for linked list is incremented." */
      curr_txbd++;
      /*SAFETYMCUSW 94 S MR:11.1,11.2,11.4 <APPROVED> "Linked List pointer needs to be assigned." */
      /*SAFETYMCUSW 95 S MR:11.1,11.4 <APPROVED> "Linked List pointer needs to be assigned." */
      /*SAFETYMCUSW 344 S MR:11.5 <APPROVED> "Linked List pointer needs to be assigned to a different structure." */      
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
      rxch_dma->active_head = (volatile emac_rx_bd_t *)curr_txbd;

      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */         
      rxch_dma->free_head = NULL;
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
      curr_bd = rxch_dma->active_head;
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
      last_bd = curr_bd;


      /*
      **  Static allocation of a specific number of packet buffers as specified by MAX_RX_PBUF_ALLOC, whose value is entered by the user in  HALCoGen GUI.
      */

      /*Commented part of allocation of pbufs need to check whether its true*/

      for(pbuf_cnt = 0U;pbuf_cnt < MAX_RX_PBUF_ALLOC;pbuf_cnt++)
        {
         /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */  
         p = pbuf_array[pbuf_cnt];
         /*SAFETYMCUSW 439 S MR:11.3 <APPROVED> "RHS is a pointer value required to be stored. - Advisory as per MISRA" */
         /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
         curr_bd->bufptr = (uint32)p;
	 memset(curr_bd->bufptr,0xff,MAX_TRANSFER_UNIT);
         /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */  
         curr_bd->bufoff_len = MAX_TRANSFER_UNIT;
         /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
         curr_bd->flags_pktlen = EMAC_BUF_DESC_OWNER;
         if (pbuf_cnt == (MAX_RX_PBUF_ALLOC - 1U))
         {
             /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */  
	   curr_bd->next = rxch_dma->active_head;//NULL;
             /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */              
             last_bd = curr_bd;

         }
         else
         {
             /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Struct pointer used for linked list is incremented." */
             curr_bd->next = (curr_bd + 1U);
             /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Struct pointer used for linked list is incremented." */
             curr_bd++;
             /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */  
             last_bd = curr_bd;
         }
       }
      
      /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */         
      // last_bd->next = NULL;
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
      rxch_dma->active_tail = last_bd;
}
#endif
#ifdef SENDULAN
tms570_eth_hw_set_RX_HDP(hdkif_t *hdkif, volatile struct emac_rx_bd *new_head)
{
  /* Writes to RX HDP are allowed
                                                 * only when it is 0
                                                 */
  while (TMS570_EMACM.RXHDP[EMAC_CHANNELNUMBER] != 0) {
    printf("HW -RX- is slacking!!!\n");
//    sys_arch_delay(10);
  }
  printf("setting RX HDP");
  EMACRxHdrDescPtrWrite(
    hdkif->emac_base,
    (uint32)new_head,
    EMAC_CHANNELNUMBER);
}
tms570_eth_hw_set_TX_HDP(hdkif_t *hdkif, volatile struct emac_tx_bd *new_head)
{
  /* Writes to RX HDP are allowed
                                                 * only when it is 0
                                                 */
//  while (hdkif->emac_base.TXHDP[EMAC_CHANNELNUMBER] != 0) {
  while (TMS570_EMACM.RXHDP[EMAC_CHANNELNUMBER] != 0) {
    printf("HW -TX- is slacking!!!\n");
//    sys_arch_delay(10);
  }
  printf("setting TX HDP");
  EMACTxHdrDescPtrWrite(
    hdkif->emac_base,
    (uint32)new_head,
    EMAC_CHANNELNUMBER);
}
#endif /*SENDULAN*/

boolean tms570_eth_send_raw(hdkif_t *hdkif, struct mbuf *mbuf)
{
  struct mbuf *q;
  struct txch *txch;
  unsigned int pktlen;
  unsigned int padlen = 0;
  volatile struct emac_tx_bd *curr_bd;
  volatile struct emac_tx_bd *packet_head;
  volatile struct emac_tx_bd *packet_tail;

//  nf_state = (struct tms570_netif_state *)netif->state;
//  txch = &(nf_state->txch);
  txch = &(hdkif->txch);

  /* Get the first BD that is unused and will be used for TX */
  curr_bd = txch->inactive_head;
  if (curr_bd == NULL)
    goto error_out_of_descriptors;

  packet_head = curr_bd;
  packet_tail = curr_bd;

  /* adjust the packet length if less than minimum required */
  pktlen = mbuf->m_pkthdr.len;
  if (pktlen < MIN_PKT_LEN) {
    padlen = MIN_PKT_LEN - pktlen;
    pktlen = MIN_PKT_LEN;
  }

  /* First 'part' of packet flags */
  curr_bd->flags_pktlen = pktlen | EMAC_BUF_DESC_SOP |
                          EMAC_BUF_DESC_OWNER;

  /* Copy pbuf information into TX BDs --
   * remember that the pbuf for a single packet might be chained!
   */
  for (q = mbuf; q != NULL; q = q->m_next) {
    if (curr_bd == NULL)
      goto error_out_of_descriptors;

    curr_bd->bufptr = mtod(q,uint8_t *);
    curr_bd->bufoff_len = (q->m_len) & 0xFFFF;

    /* This is an extra field that is not par of the in-HW BD.
     * This is used when freeing the pbuf after the TX processing
     * is done in EMAC
     */
    curr_bd->mbuf = mbuf;
    packet_tail = curr_bd;
    curr_bd = curr_bd->next;
  }
  if (padlen) {
    if (curr_bd == NULL)
      goto error_out_of_descriptors;

    /* If the ETHERNET packet is smaller than 64 bytes, it has
     * to be padded. We need some data and do not want to leak
     * random memory. Reuse IP and possibly TCP/UDP header
     * of given frame as padding
     */
    curr_bd->bufptr = packet_head->bufptr;
    curr_bd->bufoff_len = padlen;
    curr_bd->mbuf = mbuf;
    packet_tail = curr_bd;
    curr_bd = curr_bd->next;
  }
  /* Indicate the end of the packet */
  packet_tail->next = NULL;
  packet_tail->flags_pktlen |= EMAC_BUF_DESC_EOP;

  txch->inactive_head = curr_bd;
  if (curr_bd == NULL)
    txch->inactive_tail = curr_bd;

  //sys_arch_data_sync_barier();

  if (txch->active_tail == NULL) {
    txch->active_head = packet_head;
    tms570_eth_hw_set_TX_HDP(hdkif, packet_head);
  } else {
    /* Chain the bd's. If the DMA engine already reached the
     * end of the chain, the EOQ will be set. In that case,
     * the HDP shall be written again.
     */
    txch->active_tail->next = packet_head;
    curr_bd = txch->active_tail;

    /* We were too slow and the EMAC already read the
     * 'pNext = NULL' of the former txch->active_tail. In this
     * case the transmission stopped and we need to write the
     * pointer to newly added BDs to the TX HDP
     */
    if (curr_bd->flags_pktlen & EMAC_BUF_DESC_EOQ) {
      tms570_eth_hw_set_TX_HDP(hdkif, packet_head);
    }
  }
  txch->active_tail = packet_tail;

  return TRUE;

error_out_of_descriptors:
  m_freem((struct mbuf *)mbuf);//pbuf_free(pbuf);
  return FALSE;
}
/* EMAC Packet Buffer Sizes and Placement */
//#ifdef CONFIG_EMAC_PKT_FRAG_SIZE
//  #define EMAC_FRAG_SIZE    CONFIG_EMAC_PKT_FRAG_SIZE
//#else
//  #define EMAC_FRAG_SIZE      1536
//#endif
//
//#ifdef CONFIG_EMAC_ETH_FRAME_SIZE
//  #define EMAC_MAX_FLEN    CONFIG_EMAC_ETH_FRAME_SIZE
//#else
//  #define EMAC_MAX_FLEN       1536
//#endif
//
//#ifdef CONFIG_EMAC_NUM_RX_FRAGS
//  #define EMAC_NUM_RX_FRAG    CONFIG_EMAC_NUM_RX_FRAGS
//#else
//  #define EMAC_NUM_RX_FRAG    4
//#endif
//
//#ifdef CONFIG_EMAC_NUM_TX_FRAGS
//  #define EMAC_NUM_TX_FRAG    CONFIG_EMAC_NUM_TX_FRAGS
//#else
//  #define EMAC_NUM_TX_FRAG    2
//#endif
//
//#include <lwip/netifapi.h>
#ifdef	ULAN_RECV 
#endif /*ULAN_RECV*/
#ifdef ULAN
boolean EMACTransmit(hdkif_t *hdkif, pbuf_t *pbuf)
{
    
  txch *txch;
  pbuf_t *q;
  uint16 totLen;
  uint16 qLen;
  volatile emac_tx_bd_t *curr_bd,*active_head, *bd_end;
  boolean retValue = FALSE;
  if((pbuf != NULL) && (hdkif != NULL))
  {
  txch = &(hdkif->txchptr);

  /* Get the buffer descriptor which is free to transmit */
  /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
  curr_bd = txch->free_head;
  /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
  bd_end = curr_bd;
  /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
  active_head = curr_bd;

  /* Update the total packet length */
  /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */       
  curr_bd->flags_pktlen &= (~((uint32)0xFFFFU));
  /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
  totLen = pbuf->tot_len;
  curr_bd->flags_pktlen |= (uint32)(totLen);

  /* Indicate the start of the packet */
  /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
  curr_bd->flags_pktlen |= (EMAC_BUF_DESC_SOP | EMAC_BUF_DESC_OWNER);


  /* Copy pbuf information into TX buffer descriptors */
    q = pbuf;
    while(q != NULL)
    {
    /* Initialize the buffer pointer and length */
    /*SAFETYMCUSW 439 S MR:11.3 <APPROVED> "RHS is a pointer value required to be stored. - Advisory as per MISRA" */
    /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */   
    curr_bd->bufptr = (uint32)(q->payload);
    /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */
    qLen = q->len;
    curr_bd->bufoff_len = ((uint32)(qLen) & 0xFFFFU);
    /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */      
    bd_end = curr_bd;
    /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */   
    curr_bd = curr_bd->next;
    q = q->next;
    }


  /* Indicate the start and end of the packet */
  /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
  /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */       
  bd_end->next = NULL;
  /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
  bd_end->flags_pktlen |= EMAC_BUF_DESC_EOP;

  /*SAFETYMCUSW 71 S MR:17.6 <APPROVED> "Assigned pointer value has required scope." */
  txch->free_head = curr_bd;

  /* For the first time, write the HDP with the filled bd */
  if(txch->active_tail == NULL) {
    /*SAFETYMCUSW 439 S MR:11.3 <APPROVED> "Address stored in pointer is passed as as an int parameter. - Advisory as per MISRA" */
    /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */      
    EMACTxHdrDescPtrWrite(hdkif->emac_base, (uint32)(active_head), (uint32)EMAC_CHANNELNUMBER);
  }

  /*
   * Chain the bd's. If the DMA engine, already reached the end of the chain,
   * the EOQ will be set. In that case, the HDP shall be written again.
   */
  else {			

    /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */   
    curr_bd = txch->active_tail;
    /* Wait for the EOQ bit is set */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Hardware status bit read check" */
    /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
    /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */   
    while (EMAC_BUF_DESC_EOQ != (curr_bd->flags_pktlen) & EMAC_BUF_DESC_EOQ)
    {
    }
    /* Don't write to TXHDP0 until it turns to zero */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Hardware status bit read check" */
    /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
	rtems_event_set events;
				rtems_bsdnet_event_receive (INTERRUPT_EVENT,
						RTEMS_WAIT|RTEMS_EVENT_ANY,
						RTEMS_NO_TIMEOUT,
						&events);
    while (((uint32)0U != *((uint32 *)0xFCF78600U)))
    {
    }
    /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */   
    curr_bd->next = active_head;
    /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
    /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */      
    if (EMAC_BUF_DESC_EOQ == (curr_bd->flags_pktlen & EMAC_BUF_DESC_EOQ)) {
      /* Write the Header Descriptor Pointer and start DMA */
      /*SAFETYMCUSW 439 S MR:11.3 <APPROVED> "Address stored in pointer is passed as as an int parameter. - Advisory as per MISRA" */
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
      EMACTxHdrDescPtrWrite(hdkif->emac_base, (uint32)(active_head), (uint32)EMAC_CHANNELNUMBER);
    }
  }
   
  /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
  txch->active_tail = bd_end;
  retValue = TRUE;
  }
  else
  {
    retValue = FALSE;
  }
  return retValue;
}
#endif /* !ULAN */
/*
 * SCC1 interrupt handler
 */
static rtems_isr
tms_tx_interrupt_handler (rtems_vector_number v)
{  hdkif_t *hdkif;
  hdkif=&hdkif_data[0];
  tms[0].txInterrupts++;
  tms570_eth_process_irq_tx(hdkif);
//  EMACTxIntHandler(hdkif);
//  EMACCoreIntAck(EMAC_0_BASE, (uint32)EMAC_INT_CORE0_TX); 
  rtems_bsdnet_event_send (tms[0].txDaemonTid, INTERRUPT_EVENT);
  // tries=2;
  //  printf("f");           
}

static void
tms570_eth_process_irq_rx(void *arg)
{
  struct tms_softc *sc = (struct tms_softc *)arg;
  struct ifnet *ifp = &sc->arpcom.ac_if;
  hdkif_t *hdkif;
  hdkif=sc->hdkif;
//  struct tms570_netif_state *nf_state;
  struct rxch *rxch;
//  struct netif *netif = (struct netif *)arg;
  volatile struct emac_rx_bd *curr_bd;
  struct mbuf *mbuf;
  struct mbuf *q;
  struct ether_header *eh;

//  nf_state = netif->state;
//  rxch = &(nf_state->rxch);
  rxch = &(hdkif->rxch);
  /* Get the bd which contains the earliest filled data */
  curr_bd = rxch->active_head;
  if (curr_bd == NULL) {
    tms570_eth_rx_pbuf_refill(sc, 0);
    return;
  }

  /* For each valid frame */
  while ((curr_bd->flags_pktlen & EMAC_BUF_DESC_SOP) &&
         !(curr_bd->flags_pktlen & EMAC_BUF_DESC_OWNER)) {
    unsigned int total_rx_len;
    unsigned int processed_rx_len = 0;
    int corrupt_fl = 0;

//    //sys_arch_data_sync_barier();

    mbuf = curr_bd->mbuf;
    total_rx_len = curr_bd->flags_pktlen & 0xFFFF;
   // tms570_eth_debug_printf("recieve packet. L = %d ", total_rx_len);
    /* The received frame might be fragmented into muliple
     * pieces -- each one referenced by a separate BD.
     * To further process the data, we need to 'make' a
     * proper PBUF out of it -- that means linking each
     * buffer together, copy the length information form
     * the DB to PBUF, calculate the 'tot_len' etc.
     */
    for (;; ) {
      q = curr_bd->mbuf;
      eh = mtod (q, struct ether_header *);
      /* Since this pbuf will be freed, we need to
       * keep track of its size to be able to
       * allocate it back again
       */
      rxch->freed_pbuf_len += q->m_len;
//      tms570_eth_debug_printf("bd - %d ", tms570_eth_debug_get_BD_num(curr_bd, nf_state));
//      tms570_eth_debug_printf("pbuf len - %d ", q->len);
//      tms570_eth_debug_printf("A - 0x%08x ", q);
      /* This is the size of the "received data" not the PBUF */
      q->m_pkthdr.len = total_rx_len - processed_rx_len;
      q->m_len = curr_bd->bufoff_len & 0xFFFF;
      q->m_data=curr_bd->bufptr;

      if (curr_bd->flags_pktlen & EMAC_BUF_DESC_EOP)
        break;
      /*
       * If we are executing here, it means this
       * packet is being split over multiple BDs
       */
//      tms570_eth_debug_printf("MB");
      /* chain the pbufs since they belong
       * to the same packet
       */
      if (curr_bd->next == NULL) {
        corrupt_fl = 1;
        break;
      }
      curr_bd = curr_bd->next;
      q->m_next = curr_bd->mbuf;

      processed_rx_len += q->m_len;
    }
 //   tms570_eth_debug_printf("\n");
    /* Close the chain */
    q->m_next = NULL;
    if (rxch->inactive_tail == NULL) {
      rxch->inactive_head = rxch->active_head;
    } else {
      rxch->inactive_tail->next = rxch->active_head;
    }
    rxch->inactive_tail = curr_bd;
    rxch->active_head = curr_bd->next;
    if (curr_bd->next == NULL)
      rxch->active_tail = NULL;
    rxch->inactive_tail->next = NULL;


//    //LINK_STATS_INC(link.recv);

    /* Process the packet */
    /* ethernet_input((struct pbuf *)pbuf, netif) */
    if (!corrupt_fl)
	ether_input (ifp, eh, mbuf);
//      if (netif->input(pbuf, netif) != ERR_OK)
//        corrupt_fl = 1;
    if (corrupt_fl) {
//      //LINK_STATS_INC(link.memerr);
//      //LINK_STATS_INC(link.drop);
     m_freem((struct mbuf *)mbuf);// pbuf_free(pbuf);
    }

    /* Acknowledge that this packet is processed */
    EMACRxCPWrite(hdkif->emac_base, 0, (unsigned int)curr_bd);

    /* The earlier PBUF chain is freed from the upper layer.
     * So, we need to allocate a new pbuf chain and update
     * the descriptors with the PBUF info.
     * Care should be taken even if the allocation fails.
     */
    tms570_eth_rx_pbuf_refill(sc, 0);
    //tms570_eth_debug_print_rxch();
    curr_bd = rxch->active_head;
    if (curr_bd == NULL) {
      return;
    }
  }
}
static void
tms570_eth_process_irq_tx(void *arg)
{
  struct tms_softc *sc = (struct tms_softc *)arg;
  hdkif_t *hdkif;
  hdkif=sc->hdkif;

  struct txch *txch;
  volatile struct emac_tx_bd *curr_bd;
  volatile struct emac_tx_bd *start_of_packet_bd;
//  struct netif *netif = (struct netif *)arg;
//  struct tms570_netif_state *nf_state;

//  nf_state = netif->state;
  txch = &(hdkif->txch);

  start_of_packet_bd = txch->active_head;
  curr_bd = txch->active_head;

  /* Traverse the list of BDs used for transmission --
   * stop on the first unused
   */
  while ((curr_bd != NULL) && (curr_bd->flags_pktlen & EMAC_BUF_DESC_SOP)) {
    /* Make sure the transmission is over */
    if (curr_bd->flags_pktlen & EMAC_BUF_DESC_OWNER) {
//      tms570_eth_debug_printf("TXthread ownership not transfered!!!!\n");
      break;
    }

    /* Find the last chunk of the packet */
    while (!(curr_bd->flags_pktlen & EMAC_BUF_DESC_EOP))
      curr_bd = curr_bd->next;

    /* Remove flags for the transmitted BDs */
    start_of_packet_bd->flags_pktlen &= (~EMAC_BUF_DESC_SOP);
    curr_bd->flags_pktlen &= (~EMAC_BUF_DESC_EOP);

    /*
* Return processed BDs to inactive list
     */
    if (txch->inactive_tail == NULL) {
      txch->inactive_head = start_of_packet_bd;
    } else {
      txch->inactive_tail->next = start_of_packet_bd;
    }
    txch->inactive_tail = curr_bd;

    /*
     * Remove BDs from active list
                       */
    txch->active_head = curr_bd->next;
    if (curr_bd->next == NULL) {
      txch->active_tail = NULL;
    }

    /*
     * Insert null element at the end of the inactive list
                       */
    txch->inactive_tail->next = NULL;


    /* Ack the Interrupt in the EMAC peripheral */
    EMACTxCPWrite(hdkif->emac_base, CHANNEL,
                  (uint32_t)curr_bd);

    /* Free the corresponding pbuf
     * Sidenote: Each fragment of the single packet points
     * to the same pbuf // FIXME is it true?
     */
    m_freem((struct mbuf *)start_of_packet_bd->mbuf); //pbuf_free(start_of_packet_bd->mbuf);

    //LINK_STATS_INC(link.xmit);

    /* Move to the next packet */
    start_of_packet_bd = txch->active_head;
    curr_bd = txch->active_head;
  }
}
static rtems_isr
tms_rx_interrupt_handler (rtems_vector_number v)
{
	hdkif_t *hdkif;
	  hdkif=&hdkif_data[0];
    tms[0].rxInterrupts++;
    tms570_eth_process_irq_rx(hdkif);
//    EMACCoreIntAck(EMAC_0_BASE, (uint32)EMAC_INT_CORE0_RX); 
    rtems_bsdnet_event_send (tms[0].rxDaemonTid, INTERRUPT_EVENT);
  

}

static void tms570_eth_rx_pbuf_refill( struct tms_softc *sc, int single_fl)
{
  struct ifnet *ifp = &sc->arpcom.ac_if;
  hdkif_t *hdkif;
  hdkif=sc->hdkif;
  struct rxch *rxch;
  volatile struct emac_rx_bd *curr_bd;
  volatile struct emac_rx_bd *curr_head;
//  struct pbuf *new_pbuf;
  struct mbuf *new_mbuf;
  struct mbuf *m;
//  struct pbuf *q;
  uint32_t alloc_rq_bytes;
  uint16_t rxBds=0;

  //rxch = &(nf_state->rxch);
  rxch = &(hdkif->rxch);
  if (single_fl) {
    alloc_rq_bytes = PBUF_POOL_BUFSIZE;
  } else {
    alloc_rq_bytes = rxch->freed_pbuf_len;
  }

  for (; (rxch->freed_pbuf_len > 0) && (rxch->inactive_head != NULL); ) {
    //stats_display();

    curr_bd = rxch->inactive_head;
    curr_head = rxch->inactive_head;
//    tms570_eth_debug_printf("attempt to allocate %d bytes from pbuf pool (RX)\n", alloc_rq_bytes);

    MGETHDR (new_mbuf, M_WAIT, MT_DATA);
    MCLGET (new_mbuf, M_WAIT);
    new_mbuf->m_pkthdr.rcvif = ifp;
    sc->rxMbuf[rxBds] = new_mbuf;

//    new_pbuf = pbuf_alloc(PBUF_RAW,
//                          alloc_rq_bytes,
//                          PBUF_POOL);
    if (new_mbuf == NULL) {
      alloc_rq_bytes = (1 << (30-__builtin_clz(alloc_rq_bytes)));
      if (alloc_rq_bytes <= PBUF_POOL_BUFSIZE) {
        tms570_eth_debug_printf("not enough memory\n");
        break;
      }
      alloc_rq_bytes = rxch->freed_pbuf_len > alloc_rq_bytes ?
                       alloc_rq_bytes : rxch->freed_pbuf_len;
      continue;
    } else {
      m = new_mbuf;
      for (;; ) {
        curr_bd->bufptr = (uint8_t *)m->m_data;
        curr_bd->bufoff_len = m->m_len;
        curr_bd->flags_pktlen = EMAC_BUF_DESC_OWNER;
        curr_bd->mbuf = m;
        rxch->freed_pbuf_len -= m->m_len;
        m = m->m_next;
        if (m == NULL)
          break;
        if (curr_bd->next == NULL) {
          rxch->inactive_tail = NULL;
          break;
        }
        curr_bd = curr_bd->next;
      }

      if (m != NULL)
        m_freem((struct mbuf *)m);
      /* Add the newly allocated BDs to the
       * end of the list
       */
      rxch->inactive_head = curr_bd->next;

      curr_bd->next = NULL;
//      //sys_arch_data_sync_barier();

      if (rxch->active_head == NULL) {
        rxch->active_head = curr_head;
        rxch->active_tail = curr_bd;
        tms570_eth_hw_set_RX_HDP(hdkif, rxch->active_head);
      } else {
        rxch->active_tail->next = curr_head;
        //sys_arch_data_sync_barier();
        if ((rxch->active_tail->flags_pktlen & EMAC_BUF_DESC_EOQ) != 0)
          tms570_eth_hw_set_RX_HDP(hdkif, rxch->active_head);
        rxch->active_tail = curr_bd;
      }
    }
  rxBds++;
  }
}


/*
 * SCC reader task
 */
#ifndef SENDULAN
static void
tms_rxDaemon (void *arg)
{

rxch_t *rxch_int;
volatile emac_rx_bd_t *curr_bd, *curr_tail, *last_bd;
int rxBds;
struct tms_softc *sc = (struct tms_softc *)arg;
struct ifnet *ifp = &sc->arpcom.ac_if;
hdkif_t *hdkif;
struct mbuf *m;


hdkif=sc->hdkif;
rxch_int = &(sc->hdkif->rxchptr);
/*
	 * Allocate space for incoming packets and start reception
*/
for (rxBds = 0 ; ;) {
	  MGETHDR (m, M_WAIT, MT_DATA);
   	  MCLGET (m, M_WAIT);
	  m->m_pkthdr.rcvif = ifp;
	  sc->rxMbuf[rxBds] = m;
	  if (++rxBds == MAX_RX_PBUF_ALLOC-1) {
	       break;
       }
       
}
rxBds=0;
 
	//	sc->rxMbuf[rxBds]->m_data=rxch_int->active_head->bufptr;
	/*
	 * Input packet handling loop
	 */
for (;;) {
                
		/*
		 * Wait for packet if there's not one ready
		 */

		if ((*(uint32_t *)(0xFCF780A4))== 0x00000000) {
			while ((*(uint32_t *)(0xFCF780A4)) == 0x00000000 ) {
      	    	     	rtems_event_set events;
     	    		rtems_interrupt_level level;
			/* 			rtems_interrupt_disable (level);
						rtems_interrupt_enable (level);*/

				/*
				 * Unmask RXF (Full frame received) event
				 */
			
				rtems_bsdnet_event_receive (INTERRUPT_EVENT,
						RTEMS_WAIT|RTEMS_EVENT_ANY,
						RTEMS_NO_TIMEOUT,
						&events);
				
			}
		}

			struct ether_header *eh;	


  /* Get the buffer descriptors which contain the earliest filled data */

  curr_bd = rxch_int->active_head;

  last_bd = rxch_int->active_tail;

  /**
   * Process the descriptors as long as data is available
   * when the DMA is receiving data, SOP flag will be set
  */

  while((curr_bd->flags_pktlen & EMAC_BUF_DESC_SOP) == EMAC_BUF_DESC_SOP) {

    /* Start processing once the packet is loaded */

	  if((curr_bd->flags_pktlen & EMAC_BUF_DESC_OWNER)
       != EMAC_BUF_DESC_OWNER) {

      /* this bd chain will be freed after processing */

		  rxch_int->free_head = curr_bd;

      /* Get the total length of the packet. curr_bd points to the start
       * of the packet.
       */

      /* 
       * The loop runs till it reaches the end of the packet.
       */
	      
     while((curr_bd->flags_pktlen & EMAC_BUF_DESC_EOP)!= EMAC_BUF_DESC_EOP)
      {
        curr_bd->flags_pktlen = (uint32)EMAC_BUF_DESC_OWNER;
        curr_bd->bufoff_len = (uint32)MAX_TRANSFER_UNIT;
        last_bd = curr_bd;
        curr_bd = curr_bd->next;

        m = sc->rxMbuf[rxBds];//------------------
        m->m_data=last_bd->bufptr;
       	eh = mtod (m, struct ether_header *);
	m->m_len=m->m_pkthdr.len=MAX_TRANSFER_UNIT-sizeof(struct ether_header);
	m->m_data+=sizeof(struct ether_header);
	ether_input (ifp, eh, m);

	MGETHDR (m, M_WAIT, MT_DATA);
	MCLGET (m, M_WAIT);
	m->m_pkthdr.rcvif = ifp;     
      	sc->rxMbuf[rxBds] = m;	//---------------------------------------------------------
 	if(++rxBds == MAX_RX_PBUF_ALLOC-1)
	  rxBds=0;
		      
      }
      

      curr_bd->flags_pktlen = (uint32)EMAC_BUF_DESC_OWNER;

      curr_bd->bufoff_len = (uint32)MAX_TRANSFER_UNIT;

      last_bd = curr_bd;

      curr_bd = curr_bd->next;
     
      //    EMACRxCPWrite(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER, (uint32)last_bd);

      m = sc->rxMbuf[rxBds];
      //      while(last_bd->flags_pktlen&EMAC_BUF_DESC_OWNER==EMAC_BUF_DESC_OWNER)
      //	{
	  //wait for it to be ready
      //	}
        m->m_data=last_bd->bufptr;
	eh = mtod (m, struct ether_header *);
	m->m_len=m->m_pkthdr.len=MAX_TRANSFER_UNIT-sizeof(struct ether_header);//last_bd->flags_pktlen&(0x0000FFFF)-sizeof(struct ether_header);//MAX_TRANSFER_UNIT-sizeof(struct ether_header);
	m->m_data+=sizeof(struct ether_header);
	ether_input (ifp, eh, m);

	EMACRxCPWrite(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER, (uint32)last_bd);

	MGETHDR (m, M_WAIT, MT_DATA);
	MCLGET (m, M_WAIT);
	m->m_pkthdr.rcvif = ifp;      
	//	m->m_data=rxDataBuf[rxBds];
   	sc->rxMbuf[rxBds] = m;

   	if(++rxBds == MAX_RX_PBUF_ALLOC-1)
			rxBds=0;
	    
      rxch_int->active_head = curr_bd;
      curr_tail = rxch_int->active_tail;
      curr_tail->next = rxch_int->free_head;
      last_bd->next = NULL;


        /**
         * Check if the reception has ended. If the EOQ flag is set, the NULL
         * Pointer is taken by the DMA engine. So we need to write the RX HDP
         * with the next descriptor.
         */

        if((curr_tail->flags_pktlen & EMAC_BUF_DESC_EOQ) == EMAC_BUF_DESC_EOQ) {

          EMACRxHdrDescPtrWrite(hdkif->emac_base, (uint32)(rxch_int->free_head), (uint32)EMAC_CHANNELNUMBER);
        }

  
        rxch_int->free_head  = curr_bd;
        rxch_int->active_tail = last_bd;
		
	/*	if(curr_bd == 0xfc5210b0){
	  rxch_int->active_head=0xfc521010;
	   EMACRxHdrDescPtrWrite(hdkif->emac_base, (uint32)rxch_int->active_head, (uint32)EMAC_CHANNELNUMBER);
	}
	*/
       
    }
  }
        EMACCoreIntAck(EMAC_0_BASE, (uint32)EMAC_INT_CORE0_RX);
	
}
}
#endif

static void
sendpacket (struct ifnet *ifp, struct mbuf *m)
{
  uint8 *myframe;
  uint8 test[1514];
  struct pbuf_struct ti_buffer[24];
  struct mbuf *a,*b;
  bool i=false;
  int im=0,diff_min=0,acc=0;
  struct tms_softc *sc = ifp->if_softc;
   // 
  a=m;
  /*  if(a->m_ext.ext_buf!=a->m_data && (a->m_flags&M_EXT)==M_EXT)
     a->m_data=a->m_ext.ext_buf;
     ti_buffer[im].tot_len=a->m_pkthdr.len;*/

  if (m->m_pkthdr.len < MIN_PKT_LEN) {
    m->m_pkthdr.len = MIN_PKT_LEN;
    m->m_len = MIN_PKT_LEN;
  }
//  while(a){   
//        myframe=mtod(a,uint8 *);
//        if (a->m_len) {
//	  memcpy(&test[0]+acc,myframe,a->m_len);
//	  acc+=a->m_len;
//	}
//       MFREE (a, b);
//       a=b;
//  }
//  ti_buffer[im].payload=&test[0];
//  ti_buffer[im].next=NULL;
//  if(acc<MIN_FRAME_LENGTH)
//    ti_buffer[im].tot_len=ti_buffer[im].len=65;
//  else
//    ti_buffer[im].tot_len=ti_buffer[im].len=acc;
//#ifdef ULAN
    i=tms570_eth_send_raw(sc->hdkif, m);// &ti_buffer[0]);
//#else
//    i=EMACTransmit(sc->hdkif, &ti_buffer[0]);
//#endif
    //    memset(test,0,1514);
    //  if(i==false)
    //  printf("Could not send frame!");
    // else
    //   printf("good");

    
}

/*
 * Driver transmit daemon
 */
void
tms_txDaemon (void *arg)
{
	struct tms_softc *sc = (struct tms_softc *)arg;
	struct ifnet *ifp = &sc->arpcom.ac_if;
	struct mbuf *m;
	rtems_event_set events;
	int i;
	char *mo;
	for (;;) {
		/*
		 * Wait for packet
		 */
	  		rtems_bsdnet_event_receive (START_TRANSMIT_EVENT, RTEMS_EVENT_ANY | RTEMS_WAIT, RTEMS_NO_TIMEOUT, &events);

		/*
		 * Send packets till queue is empty
		 */
		for (;;) {
			/*
			 * Get the next mbuf chain to transmit.
			 */
			IF_DEQUEUE(&ifp->if_snd, m);
			if (!m)
				break;
			sendpacket (ifp, m);
		}
		ifp->if_flags &= ~IFF_OACTIVE;
	}
}

/*
 * Send packet (caller provides header).
 */
static void
tms_start (struct ifnet *ifp)
{
	struct tms_softc *sc = ifp->if_softc;

	rtems_bsdnet_event_send (sc->txDaemonTid, START_TRANSMIT_EVENT);
	ifp->if_flags |= IFF_OACTIVE;
}


static void
tms_EMAC_hw_init(uint8 macaddr[6U])
{
  rtems_status_code status;

  uint32 temp, channel;
  volatile uint32 phyID=0U;
  volatile uint32 delay = 0xFFFU;
  uint32 phyIdReadCount = 0xFFFFU;
  volatile uint32 phyLinkRetries = 0xFFFFU;
  hdkif_t *hdkif;
  rxch_t *rxch;
  uint32 retVal = EMAC_ERR_OK;
  uint32 emacBase = 0U;

  hdkif = &hdkif_data[0U];
  EMACInstConfig(hdkif);
  /* set MAC hardware address */
//  for(temp = 0U; temp < EMAC_HWADDR_LEN; temp++) {
//    hdkif->mac_addr[temp] = macaddr[(EMAC_HWADDR_LEN - 1U) - temp];
//  }
  /*Initialize the EMAC, EMAC Control and MDIO modules. */
  EMACInit(hdkif->emac_ctrl_base, hdkif->emac_base);
  MDIOInit(hdkif->mdio_base, MDIO_FREQ_INPUT, MDIO_FREQ_OUTPUT);
  
  /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
  while(delay != 0U)
  {
  /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
    delay--;
  }
  
  /* Set the MAC Addresses in EMAC hardware */
  //emacBase = hdkif->emac_base; /* MISRA Code Fix (12.2) */
  //HWREG(emacBase + EMAC_MACINDEX) =  (uint32)EMAC_CHANNELNUMBER;
  // for(channel = 0U; channel < 8U; channel++) {
    //    emacBase = hdkif->emac_base;
    //   EMACMACSrcAddrSet(emacBase,hdkif->mac_addr);
       //EMACMACAddrSet(emacBase, channel, hdkif->mac_addr, EMAC_MACADDR_MATCH);


  //}
  emacBase = hdkif->emac_base; /* MISRA Code Fix (12.2) */
  EMACMACSrcAddrSet(emacBase, hdkif->mac_addr);
  for(channel = 0U; channel < 8U; channel++) {
       emacBase = hdkif->emac_base;
       EMACMACAddrSet(emacBase, channel, emacBase, EMAC_MACADDR_MATCH);
  }
  
  


  /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
  while ((phyID == 0U) && (phyIdReadCount > 0U)) {
      phyID = Dp83640IDGet(hdkif->mdio_base,hdkif->phy_addr);
      phyIdReadCount--;
  }

  if (0U == phyID) {
      retVal = EMAC_ERR_CONNECT;
  } else {

  }

  if((uint32)0U == ((MDIOPhyAliveStatusGet(hdkif->mdio_base)
        >> hdkif->phy_addr) & (uint32)0x01U )) {
    retVal = EMAC_ERR_CONNECT;
  } else {

  }


  if(!Dp83640LinkStatusGet(hdkif->mdio_base, (uint32)EMAC_PHYADDRESS, (uint32)phyLinkRetries)) {
      retVal = EMAC_ERR_CONNECT;
  } else {

  }

  if(EMACLinkSetup(hdkif) != EMAC_ERR_OK) {
    retVal = EMAC_ERR_CONNECT;
  } else {

  }

  /* The transmit and receive buffer descriptors are initialized here. 
   * Also, packet buffers are allocated to the receive buffer descriptors.
   */
#ifdef SENDULAN
tms570_eth_init_buffer_descriptors(hdkif);
#else
  EMACDMAInit(hdkif);
#endif
  /* Acknowledge receive and transmit interrupts for proper interrupt pulsing*/
  EMACCoreIntAck(hdkif->emac_base, (uint32)EMAC_INT_CORE0_RX);
  EMACCoreIntAck(hdkif->emac_base, (uint32)EMAC_INT_CORE0_TX);

  /* Enable MII if enabled in the GUI. */
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
//#if(EMAC_MII_ENABLE)
//      EMACMIIEnable(hdkif->emac_base);
//#else
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */  
      EMACMIIDisable(hdkif->emac_base);
//#endif
  EMACRMIISpeedSet(hdkif->emac_base,EMAC_RMIISPEED_100MBPS);


  /* Enable Broadcast if enabled in the GUI. */
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
//#if(EMAC_BROADCAST_ENABLE)
      EMACRxBroadCastEnable(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER);
      HWREG(emacBase + EMAC_RXMBPENABLE) |= ((uint32)0x20  );
//#else
   /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */
   /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */
//      EMACRxBroadCastDisable(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER);
//#endif
  
  /* Enable Broadcast if enabled in the GUI. */
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
#if(EMAC_UNICAST_ENABLE)
      EMACRxUnicastSet(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER);
#else
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */
      EMACRxUnicastClear(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER);
#endif

  /*Enable Full Duplex or Half-Duplex mode based on GUI Input. */
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
//#if (EMAC_FULL_DUPLEX_ENABLE)
        EMACDuplexSet(EMAC_0_BASE, (uint32)EMAC_DUPLEX_FULL);
//#else
        /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition arameter is taken as input from GUI." */
//        EMACDuplexSet(EMAC_0_BASE, (uint32)EMAC_DUPLEX_HALF);
//#endif

  /* Enable Loopback based on GUI Input */ 
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
#if(EMAC_LOOPBACK_ENABLE)
      EMACEnableLoopback(hdkif->emac_base);
#else
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */  
      EMACDisableLoopback(hdkif->emac_base);
#endif

  /* Enable Transmit and Transmit Interrupt */ 
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
//#if(EMAC_TX_ENABLE)
      EMACTxEnable(hdkif->emac_base);
      EMACTxIntPulseEnable(hdkif->emac_base, hdkif->emac_ctrl_base, (uint32)EMAC_CHANNELNUMBER, (uint32)EMAC_CHANNELNUMBER);
//#else
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */
  //    EMACTxDisable(hdkif->emac_base);
  //    EMACTxIntPulseDisable(hdkif->emac_base, hdkif->emac_ctrl_base, (uint32)EMAC_CHANNELNUMBER, (uint32)EMAC_CHANNELNUMBER);     
//#endif

  /* Enable Receive and Receive Interrupt. Then start receiving by writing to the HDP register. */
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
//#if(EMAC_RX_ENABLE)
//      EMACNumFreeBufSet(hdkif->emac_base,(uint32)EMAC_CHANNELNUMBER , (uint32)MAX_RX_PBUF_ALLOC);
      EMACRxEnable(hdkif->emac_base);
      EMACRxIntPulseEnable(hdkif->emac_base, hdkif->emac_ctrl_base, (uint32)EMAC_CHANNELNUMBER, (uint32)EMAC_CHANNELNUMBER);
      rxch = &(hdkif->rxchptr);
      /* Write to the RX HDP for channel 0 */
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
   //   EMACRxHdrDescPtrWrite(hdkif->emac_base, (uint32)rxch->active_head, (uint32)EMAC_CHANNELNUMBER);
//#else
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */    
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */  
//      EMACRxDisable(hdkif->emac_base);
//      EMACRxIntPulseDisable(hdkif->emac_base, hdkif->emac_ctrl_base, (uint32)EMAC_CHANNELNUMBER, (uint32)EMAC_CHANNELNUMBER);     
//#endif
      *(uint32_t *)(0xFCF78100)|=0x00800000;//0x00A00000;
      //      *(uint32_t *)(0xFCF78100)&=0xFFFFDFFF;//0x00A00000;

      if(retVal==EMAC_ERR_CONNECT)
	printf("emachwinit failed...");  
    status = bsp_interrupt_vector_enable(
					  TMS570_IRQ_EMAC_TX
					   );

      status = rtems_interrupt_handler_install( TMS570_IRQ_EMAC_TX,
      "tms1",
      RTEMS_INTERRUPT_SHARED,
      tms570_eth_process_irq,
      NULL
						);
      if (status != RTEMS_SUCCESSFUL) 
	printf ("Can't install int tx handler: `%s'\n", rtems_status_text (status));


      status = bsp_interrupt_vector_enable(
					 TMS570_IRQ_EMAC_RX 
					   );

      status = rtems_interrupt_handler_install( TMS570_IRQ_EMAC_RX,
      "tms1",
      RTEMS_INTERRUPT_SHARED,
      tms570_eth_process_irq,
      NULL
						);

 if (status != RTEMS_SUCCESSFUL) 
      printf ("Can't install int rx handler: `%s'\n", rtems_status_text (status));

 tms[0].rxMbuf = malloc (tms[0].rxBdCount * sizeof *tms[0].rxMbuf, M_MBUF, M_NOWAIT);
}
/*
 * Initialize and start the device
 */


tms570_eth_process_irq(void *arg)
{
//  struct netif *netif = (struct netif *)argument;
//  struct tms570_netif_state *nf_state;
//
//  nf_state = netif->state;
  struct tms_softc *sc = arg;
  hdkif_t *hdkif;
//  struct ifnet *ifp = &sc->arpcom.ac_if;
  hdkif=sc->hdkif;
  uint32_t macints;
  while (1) {
    macints = TMS570_EMACM.MACINVECTOR;
    if ((macints & 0xffff) == 0) {
      break;
    }
    if (macints & (0xff<<16)) { //TX interrupt
      tms570_eth_process_irq_tx(sc);
      EMACCoreIntAck(hdkif->emac_base, EMAC_INT_CORE0_TX);
    }
    if (macints & (0xff<<0)) { //RX interrupt
      tms570_eth_process_irq_rx(sc);
      EMACCoreIntAck(hdkif->emac_base, EMAC_INT_CORE0_RX);
    }
  }
//  sys_arch_unmask_interrupt_source(TMS570_IRQ_EMAC_RX);
//  sys_arch_unmask_interrupt_source(TMS570_IRQ_EMAC_TX);
}


void static
tms570_eth_process_irq_request(void *arg)
{
//  struct netif *netif = (struct netif *)argument;
//  struct tms570_netif_state *nf_state;

  struct tms_softc *sc = arg;
  hdkif_t *hdkif;
//  struct ifnet *ifp = &sc->arpcom.ac_if;
  hdkif=sc->hdkif;
//  nf_state = netif->state;

  for (;; ) {
    sys_arch_sem_wait(sc->rxDaemonTid, 0);
    tms570_eth_process_irq(sc);
  }
}

u32_t
sys_arch_sem_wait(rtems_id rtid, u32_t timeout)
{
  rtems_status_code status;
  rtems_interval tps = rtems_clock_get_ticks_per_second();
  rtems_interval tick_timeout;
  uint64_t       start_time;
  uint64_t       wait_time;

  start_time = rtems_clock_get_uptime_nanoseconds();
  if (timeout == 0) {
    tick_timeout = RTEMS_NO_TIMEOUT;
  } else {
    tick_timeout = (timeout * tps + 999) / 1000;
  }
  status = rtems_semaphore_obtain(rtid, RTEMS_WAIT, tick_timeout);
  if (status == RTEMS_TIMEOUT) {
    return SYS_ARCH_TIMEOUT;
  }
  if (status != RTEMS_SUCCESSFUL) {
    return SYS_ARCH_TIMEOUT;
  }
  wait_time = rtems_clock_get_uptime_nanoseconds() - start_time;
  return wait_time / (1000 * 1000);
}
static void
tms_eth_init (void *arg)
{
	struct tms_softc *sc = arg;
        hdkif_t *hdkif;
	struct ifnet *ifp = &sc->arpcom.ac_if;
        hdkif=sc->hdkif;
        
	if (sc->txDaemonTid == 0) {

		/*
		 * Set up SCC hardware
		 */
	  tms_EMAC_hw_init(hdkif->mac_addr);

		/*
		 * Start driver tasks
		 */
//		sc->txDaemonTid = rtems_bsdnet_newproc ("TMStx", 2048, tms_txDaemon, sc);
//		sc->rxDaemonTid = rtems_bsdnet_newproc ("TMSrx", 2048, tms_rxDaemon, sc);
//		sc->txDaemonTid = rtems_bsdnet_newproc ("TMStx", 1024, tms570_eth_process_irq_request, sc);
		sc->rxDaemonTid = rtems_bsdnet_newproc ("TMSrx", 2048, tms570_eth_process_irq_request, sc);

	}

	/*
	 * Tell the world that we're running.
	 */
	ifp->if_flags |= IFF_RUNNING;

}

/*
 * Stop the device
 */
static void
tms_eth_stop (struct tms_softc *sc)
{
	struct ifnet *ifp = &sc->arpcom.ac_if;
        hdkif_t *hdkif;

	ifp->if_flags &= ~IFF_RUNNING;
        
	/*
	 * Shut down receiver and transmitter
	 */
        hdkif=sc->hdkif;
        EMACMIIDisable(hdkif->emac_base);
}

/*
 * Show interface statistics
 */
static void
tms_stats (struct tms_softc *sc)
{
	printf ("      Rx Interrupts:%-8lu", sc->rxInterrupts);
	//	printf ("       Failed Tx:%-8lu", sc->failedTX);
	/*	printf ("        Not Last:%-8lu\n", sc->rxNotLast);
	printf ("              Giant:%-8lu", sc->rxGiant);
	printf ("            Runt:%-8lu", sc->rxRunt);
	printf ("       Non-octet:%-8lu\n", sc->rxNonOctet);
	printf ("            Bad CRC:%-8lu", sc->rxBadCRC);
	printf ("         Overrun:%-8lu", sc->rxOverrun);
	printf ("       Collision:%-8lu\n", sc->rxCollision);
	printf ("          Discarded:%-8lu\n", (unsigned long)m360.scc1p.un.ethernet.disfc);*/

	printf ("      Tx Interrupts:%-8lu", sc->txInterrupts);
	/*printf ("        Deferred:%-8lu", sc->txDeferred);
	printf (" Missed Hearbeat:%-8lu\n", sc->txHeartbeat);
	printf ("         No Carrier:%-8lu", sc->txLostCarrier);
	printf ("Retransmit Limit:%-8lu", sc->txRetryLimit);
	printf ("  Late Collision:%-8lu\n", sc->txLateCollision);
	printf ("           Underrun:%-8lu", sc->txUnderrun);
	printf (" Raw output wait:%-8lu", sc->txRawWait);
	printf ("       Coalesced:%-8lu\n", sc->txCoalesced);
	printf ("    Coalesce failed:%-8lu", sc->txCoalesceFailed);
	printf ("         Retries:%-8lu\n", sc->txRetry);*/
}

/*
 * Driver ioctl handler
 */
static int
tms_cmd_ioctl (struct ifnet *ifp, ioctl_command_t command, caddr_t data)
{
	struct tms_softc *sc = ifp->if_softc;
	int error = 0;

	switch (command) {
	case SIOCGIFADDR:
	case SIOCSIFADDR:
		ether_ioctl (ifp, command, data);
		break;

	case SIOCSIFFLAGS:
		switch (ifp->if_flags & (IFF_UP | IFF_RUNNING)) {
		case IFF_RUNNING:
			tms_eth_stop (sc);
			break;

		case IFF_UP:
			tms_eth_init (sc);
			break;

		case IFF_UP | IFF_RUNNING:
			tms_eth_stop (sc);
			tms_eth_init (sc);
			break;

		default:
			break;
		}
		break;

	case SIO_RTEMS_SHOW_STATS:
		tms_stats (sc);
		break;

	/*
	 * FIXME: All sorts of multicast commands need to be added here!
	 */
	default:
		error = EINVAL;
		break;
	}
	return error;
}

/*
 * Attach an SCC driver to the system
 */
int
rtems_tms_driver_attach (struct rtems_bsdnet_ifconfig *config, int attaching)
{
	struct tms_softc *sc;
	struct ifnet *ifp;
        hdkif_t *hdkif;
  //	uint32 rval;
	//int mtu;
	int unitNumber;
	char *unitName;

	/*
	 * Make sure we're really being attached
	 */
	//configure_correct_pins();
//  rval = *((uint32 *)pinMuxBaseReg+356);
//  printf("%d\n", rval );
	if (!attaching) {
		printf ("SCC1 driver can not be detached.\n");
		return 0;
	}

	/*
 	 * Parse driver name
	 */
	if ((unitNumber = rtems_bsdnet_parse_driver_name (config, &unitName)) < 0)
		return 0;
        
        /* Zync code
         * int           unit_number             = rtems_bsdnet_parse_driver_name(
    bsd_config,
    &unit_name );
         */

	/*
	 * Is driver free?
	 */
	if ((unitNumber <= 0) || (unitNumber > NTMSDRIVER)) {
		printf ("Bad SCC unit number.\n");
		return 0;
	}
	
	sc = &tms[unitNumber - 1];
	sc->hdkif=&hdkif_data[unitNumber-1];        
	hdkif=sc->hdkif;
	ifp = &sc->arpcom.ac_if;
        /*struct ifnet *ifp                     = &self->arpcom.ac_if;*/
	if (ifp->if_softc != NULL) {
		printf ("Driver already in use.\n");
		return 0;
	}

	/*
	 * Process options
	 */
	if (config->hardware_address) {
		memcpy (sc->arpcom.ac_enaddr, config->hardware_address, ETHER_ADDR_LEN);
                memcpy (hdkif->mac_addr, config->hardware_address, ETHER_ADDR_LEN);
	}
    
 /************************Zync board
         Copy MAC address 
        
                 memcpy(
            self->arpcom.ac_enaddr,
            bsd_config->hardware_address,
            ETHER_ADDR_LEN
        );
         

       
	
	if (config->mtu)
		mtu = config->mtu;
	else
		mtu = ETHERMTU;*/
											/*
											 *Here rxBdCount represents the number of receive
											 *buffers to be allocated, normally indicated in
											 *the config structure.
											 * */
	if (config->rbuf_count)
		sc->rxBdCount = config->rbuf_count;
	else
		sc->rxBdCount = MAX_RX_PBUF_ALLOC;
											
									
	/*Same for transmit buffer.
										       */
	if (config->xbuf_count)
		sc->txBdCount = config->xbuf_count;
	else
		sc->txBdCount = TX_BUF_COUNT * TX_BD_PER_BUF;
//	sc->acceptBroadcast = !config->ignore_broadcast;

	/*
	 * Set up network interface values
	 */
	ifp->if_softc = sc;
	ifp->if_unit = unitNumber;
	ifp->if_name = unitName;
	ifp->if_init = tms_eth_init;
	ifp->if_ioctl = tms_cmd_ioctl;
	ifp->if_start = tms_start;
	ifp->if_output = ether_output;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX;
	if (ifp->if_snd.ifq_maxlen == 0)
		ifp->if_snd.ifq_maxlen = ifqmaxlen;

        /*****************Zynk board */
           /* Set interface data */
        /*
      *ifp->if_softc          = self;
      ifp->if_unit           = (short) unit_number;
      ifp->if_name           = unit_name;

      ifp->if_start          = dwmac_if_interface_start;  *//* Start transmitting
                                                             frames */
      /*ifp->if_output         = ether_output;
      ifp->if_watchdog       = NULL;
      ifp->if_flags          = IFF_BROADCAST | IFF_SIMPLEX;
      ifp->if_snd.ifq_maxlen = ifqmaxlen;
      ifp->if_timer          = 0;
      ifp->if_mtu            =
        bsd_config->mtu > 0 ? (u_long) bsd_config->mtu : ETHERMTU;
      */
	/*
	 * Attach the interface
	 */
	if_attach (ifp);
	ether_ifattach (ifp);
	return 1;
}

void configure_correct_pins(void)
{
 uint TXD0,TXD1,TXEN,CRS,CLK,RXD0,RXD1,RXER,SEL;
  TXD0 =0x34; //OFF0 44
  TXD1 =0x34;  //OFF8 44
  TXEN =0x34; //OFF16 44
  CRS  =0x44U;//OFF16 54
  CLK  =0x38;//OFF8 48
  RXD0 =0x2CU;//OFF24 3C
  RXD1 =0x30U;//OFF0 40
  RXER =0x28U;// OFF0 38
  SEL  =0x74;

 *((uint32_t *)RegEdit) = 0x83E70B13U;
  *((uint32_t *)RegEdit+1) = 0x95A4F1E0U;
 
  
  if(SET_DRIVER_RMII==0){

	*(int *) 0xFFFFEB38  &= 0xFFFFFF00; 
	*(int *) 0xFFFFEB38  |= (1 << 1);   
	
	*(int *) 0xFFFFEB3C  &= 0x00FFFFFF; 
	*(int *) 0xFFFFEB3C  |= (1 << 26);  

	*(int *) 0xFFFFEB40  &= 0x0000FF00;
	*(int *) 0xFFFFEB40  |= ((1<<26) | (1<<18) | (1<<1));

	*(int *) 0xFFFFEB44  &= 0x00000000;
	*(int *) 0xFFFFEB44  |= ((1<<26)|(1<<18)|(1<<10)|(1<<2)); 

	*(int *) 0xFFFFEB48  &= 0xFFFF0000; 
	*(int *) 0xFFFFEB48  |= ((1<<9)|(1<<2));    

	*(int *) 0xFFFFEB54  &= 0xFF00FF00      ;
	*(int *) 0xFFFFEB54  |= ((1<<17)|(1<<1));        

	*(int *) 0xFFFFEB5C  &= 0xFFFF00FF;  
	*(int *) 0xFFFFEB5C  |= (1<<9);      

	*(int *) 0xFFFFEB60  &= 0xFF00FFFF;  
	*(int *) 0xFFFFEB60  |= (1<<18);     

	*(int *) 0xFFFFEB84  &= 0x00FFFFFF;
	*(int *) 0xFFFFEB84  |= (0<<24);   
  }else {

     *((uint32_t *)pinMuxBaseReg+(TXEN/4))=0x1080808;
     *((uint32_t *)pinMuxBaseReg+(CRS/4))=0x2040200;
     *((uint32_t *)pinMuxBaseReg+(CLK/4))=0x1010401;
     *((uint32_t *)pinMuxBaseReg+(RXD0/4))=0x8020101;
     *((uint32_t *)pinMuxBaseReg+(RXD1/4))=0x1010204;
     *((uint32_t *)pinMuxBaseReg+(RXER/4))=0x2010204;
    *((uint32_t *)pinMuxBaseReg+(SEL/4))=  0x1000001;
}
  

 
  *((uint32_t *)RegEdit) = 0x00000000U;
  *((uint32_t *)RegEdit+1) = 0x00000000U;




}
