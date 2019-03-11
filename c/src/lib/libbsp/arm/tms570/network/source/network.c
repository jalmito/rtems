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
static uint8_t rxDataBuf[MAX_RX_PBUF_ALLOC][MAX_TRANSFER_UNIT];
#define NTMSDRIVER	1


/*
 * Default number of buffer descriptors set aside for this driver.
 * The number of transmit buffer descriptors has to be quite large
 * since a single frame often uses four or more buffer descriptors.
 */
#define RX_BUF_COUNT     15
#define TX_BUF_COUNT     4
#define TX_BD_PER_BUF    4
#define MIN_FRAME_LENGTH 60U
#define pinMuxBaseReg ((long unsigned int) 0xFFFF1D10U) 
#define RegEdit ((long unsigned int) 0xFFFFEA38U)
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


#ifdef ULAN
#else
void EMACDMAInit(hdkif_t *hdkif)
{

      uint32 num_bd, pbuf_cnt = 0U;
      volatile emac_tx_bd_t *curr_txbd, *last_txbd;
      txch_t *txch_dma;
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
//      last_txbd = curr_txbd;

      /* Initialize all the TX buffer Descriptors */
      while(num_bd != 0U) {
        /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Struct pointer used for linked list is incremented." */
        curr_txbd->next = (emac_tx_bd_t *)EMACSwizzleData((uint32)(curr_txbd + 1U));
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
        curr_txbd->flags_pktlen = 0U;
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */   
        last_txbd = curr_txbd;
        /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */   
        curr_txbd = (emac_tx_bd_t *)EMACSwizzleData((uint32)(curr_txbd->next));
        num_bd--;
      }
      /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
      last_txbd->next = (emac_tx_bd_t *)EMACSwizzleData((uint32)(txch_dma->free_head));

      /* Initialize the descriptors for the RX channel */
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
      rxch_dma = &(hdkif->rxchptr);
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
      /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Struct pointer used for linked list is incremented." */
//      curr_txbd++;
      /*SAFETYMCUSW 94 S MR:11.1,11.2,11.4 <APPROVED> "Linked List pointer needs to be assigned." */
      /*SAFETYMCUSW 95 S MR:11.1,11.4 <APPROVED> "Linked List pointer needs to be assigned." */
      /*SAFETYMCUSW 344 S MR:11.5 <APPROVED> "Linked List pointer needs to be assigned to a different structure." */      
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
      rxch_dma->active_head = (volatile emac_rx_bd_t *)(curr_txbd + 1);

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
         curr_bd->bufptr = EMACSwizzleData((uint32)p);
	 memset(EMACSwizzleData((uint32)curr_bd->bufptr),0xff,MAX_TRANSFER_UNIT);
         /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */  
         curr_bd->bufoff_len = EMACSwizzleData(MAX_TRANSFER_UNIT);
         /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
         curr_bd->flags_pktlen = EMACSwizzleData(EMAC_BUF_DESC_OWNER);
         if (pbuf_cnt == (MAX_RX_PBUF_ALLOC - 1U))
         {
             /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */  
	   curr_bd->next = (emac_rx_bd_t *)EMACSwizzleData((uint32)rxch_dma->active_head);//NULL;
             /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */              
             last_bd = curr_bd;

         }
         else
         {
             /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Struct pointer used for linked list is incremented." */
             curr_bd->next = (emac_rx_bd_t *)EMACSwizzleData((uint32)(curr_bd + 1U));
             /*SAFETYMCUSW 567 S MR:17.1,17.4 <APPROVED> "Struct pointer used for linked list is incremented." */
             curr_bd=(emac_rx_bd_t *)EMACSwizzleData((uint32)curr_bd->next);
             /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */  
             last_bd = curr_bd;
         }
       }
      
      /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */         
       last_bd->next = NULL;
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */          
      rxch_dma->active_tail = last_bd;
}
#endif

boolean EMACTransmit_mbuf(hdkif_t *hdkif, struct mbuf *m)
{
   
  txch_t *txch;
 // pbuf_t *q;
  uint16 totLen;
  uint16 qLen;
  struct mbuf *aux,*aux_next;

  volatile emac_tx_bd_t *curr_bd,*active_head, *bd_end;
  boolean retValue = FALSE;
  if((m != NULL) && (hdkif != NULL))
  {
  txch = &(hdkif->txchptr);

  /* Get the buffer descriptor which is free to transmit */
  curr_bd = txch->free_head;
  bd_end = curr_bd;
  active_head = curr_bd;

  /* Update the total packet length */
  uint16_t totlength=0;
  uint32 flags_pktlen;
//  flags_pktlen = 0;//m->m_len;

  if(m->m_pkthdr.len < MIN_FRAME_LENGTH)
     flags_pktlen = MIN_FRAME_LENGTH;
  else
     flags_pktlen = m->m_pkthdr.len;
  if(( m->m_flags & M_EXT ) == M_EXT )
    flags_pktlen = m->m_pkthdr.len;//m_ext.ext_size - MAX_FRAME_LENGTH;
//    flags_pktlen = aux->m_pkthdr.len;//m_ext.ext_size - MAX_FRAME_LENGTH;
//  else

  flags_pktlen |= (EMAC_BUF_DESC_SOP | EMAC_BUF_DESC_OWNER);

  curr_bd->flags_pktlen = EMACSwizzleData(flags_pktlen);


  /* Copy pbuf information into TX buffer descriptors */
//    aux=m_pullup(m,m->m_len);
    aux=m;
//    aux_next=m;
    while(aux != NULL)
    {
    /* Initialize the buffer pointer and length */
    /*SAFETYMCUSW 439 S MR:11.3 <APPROVED> "RHS is a pointer value required to be stored. - Advisory as per MISRA" */
    /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */

        if(totlength > 0 )
        {
            bd_end->flags_pktlen &= 0x0000FFFFU;

            bd_end->flags_pktlen |= EMACSwizzleData( (totlength + aux->m_len) & 0xFFFFU);
        }
        if(( aux->m_flags & M_EXT ) == M_EXT  && totlength + aux->m_len > MAX_FRAME_LENGTH)
        {
            bd_end->flags_pktlen &= 0x0000FFFFU;

            bd_end->flags_pktlen |= EMACSwizzleData( (MAX_FRAME_LENGTH - MIN_FRAME_LENGTH) & 0xFFFFU);

            curr_bd->bufptr = EMACSwizzleData((uint32)(mtod(aux,uint8_t *)));

            curr_bd->bufoff_len = EMACSwizzleData( (MAX_FRAME_LENGTH - MIN_FRAME_LENGTH) & 0xFFFFU);

//            curr_bd->flags_pktlen |= EMACSwizzleData(EMAC_BUF_DESC_EOP);
            bd_end = curr_bd;
            curr_bd = (emac_tx_bd_t *)EMACSwizzleData((uint32)curr_bd->next);


        
           flags_pktlen =  aux->m_len - MAX_FRAME_LENGTH - totlength + MIN_FRAME_LENGTH ;
           flags_pktlen |= (EMAC_BUF_DESC_SOP | EMAC_BUF_DESC_OWNER | EMAC_BUF_DESC_EOP );
//            flags_pktlen |= (EMAC_BUF_DESC_SOP | EMAC_BUF_DESC_EOP );
           curr_bd->flags_pktlen = EMACSwizzleData(flags_pktlen);

//            curr_bd->bufptr = EMACSwizzleData((uint32)(aux->m_ext.ext_buf + MAX_FRAME_LENGTH - totlength));
           curr_bd->bufptr = EMACSwizzleData((uint32)((uint8 *)(mtod(aux,uint8_t *) + totlength + MAX_FRAME_LENGTH - MIN_FRAME_LENGTH)));
//            curr_bd->bufptr = EMACSwizzleData((uint32)&vectest[0]);//EMACSwizzleData((uint32)(mtod(aux,uint8_t *)));

           curr_bd->bufoff_len = EMACSwizzleData((aux->m_len - MAX_FRAME_LENGTH - totlength + MIN_FRAME_LENGTH) & 0xFFFFU);// | (aux->m_ext.ext_size - MAX_FRAME_LENGTH) <<16);
           bd_end = curr_bd;
           curr_bd = (emac_tx_bd_t *)EMACSwizzleData((uint32)curr_bd->next);
           totlength = 0;
        }
        else
        {
            curr_bd->bufptr = EMACSwizzleData((uint32)(mtod(aux,uint8_t *)));
            curr_bd->bufoff_len = EMACSwizzleData(aux->m_len & 0xFFFFU);
            bd_end = curr_bd;
            curr_bd = (emac_tx_bd_t *)EMACSwizzleData((uint32)curr_bd->next);
            totlength += aux->m_len;
        }
//    MFREE (aux, aux_next);
//    aux=aux_next;
//    if((aux->m_next->m_flags & M_EXT ) == M_EXT  )
//    aux_next=m_pullup(aux->m_next,aux->m_next->m_len);
    aux=aux->m_next;
    }
   if( totlength > 0 && totlength < MIN_FRAME_LENGTH){
            bd_end->bufoff_len = EMACSwizzleData((uint32)MIN_FRAME_LENGTH);
            bd_end->flags_pktlen |= EMACSwizzleData((uint32)MIN_FRAME_LENGTH);
   }

  /* Indicate the start and end of the packet */
  /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
  /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */       
  bd_end->next = NULL;
  /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
  bd_end->flags_pktlen |= EMACSwizzleData(EMAC_BUF_DESC_EOP);

  /*SAFETYMCUSW 71 S MR:17.6 <APPROVED> "Assigned pointer value has required scope." */
  txch->free_head = curr_bd;

  /* For the first time, write the HDP with the filled bd */
  if(txch->active_tail == NULL) {
    EMACTxHdrDescPtrWrite(hdkif->emac_base, (uint32)(active_head), (uint32)EMAC_CHANNELNUMBER);
  }

       
         /*
          * Chain the bd's. If the DMA engine, already reached the end of the chain,
          * the EOQ will be set. In that case, the HDP shall be written again.
          */
  else { 
           curr_bd = txch->active_tail;
           /* Wait for the EOQ bit is set */
           while (EMAC_BUF_DESC_EOQ != (EMACSwizzleData(curr_bd->flags_pktlen) & EMAC_BUF_DESC_EOQ))
           {
           }
           /* Don't write to TXHDP0 until it turns to zero */
       //	rtems_event_set events;
       //				rtems_bsdnet_event_receive (INTERRUPT_EVENT,
       //						RTEMS_WAIT|RTEMS_EVENT_ANY,
       //						RTEMS_NO_TIMEOUT,
       //						&events);
           while (((uint32)0U != *((uint32 *)0xFCF78600U)))
           {
           }
           curr_bd->next = (emac_tx_bd_t *)EMACSwizzleData((uint32)active_head);
           if (EMAC_BUF_DESC_EOQ == (EMACSwizzleData(curr_bd->flags_pktlen) & EMAC_BUF_DESC_EOQ)) {
             /* Write the Header Descriptor Pointer and start DMA */
             EMACTxHdrDescPtrWrite(hdkif->emac_base, (uint32)(active_head), (uint32)EMAC_CHANNELNUMBER);
           }
    }
   
  txch->active_tail = bd_end;
  /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
  m_freem(aux_next);
  retValue = TRUE;
  }
  else
  {
    retValue = FALSE;
  }
  return retValue;
}
#ifndef ULAN
boolean EMACTransmit(hdkif_t *hdkif, pbuf_t *pbuf)
{
    
  txch_t *txch;
  pbuf_t *q;
  volatile emac_tx_bd_t *curr_bd,*active_head, *bd_end;
  boolean retValue = FALSE;
  if((pbuf != NULL) && (hdkif != NULL))
  {
  txch = &(hdkif->txchptr);

  /* Get the buffer descriptor which is free to transmit */
  curr_bd = txch->free_head;
  bd_end = curr_bd;
  active_head = curr_bd;

  /* Update the total packet length */
  uint32 flags_pktlen = pbuf->tot_len; 

  flags_pktlen |= (EMAC_BUF_DESC_SOP | EMAC_BUF_DESC_OWNER);

  curr_bd->flags_pktlen = EMACSwizzleData(flags_pktlen);

    q = pbuf;
    while(q != NULL)
    {
    /* Initialize the buffer pointer and length */
    curr_bd->bufptr = EMACSwizzleData((uint32)(q->payload));
    curr_bd->bufoff_len = EMACSwizzleData((q->len) & 0xFFFFU);
    bd_end = curr_bd;
//    curr_bd->pbuf = pbuf;
    curr_bd = (emac_tx_bd_t *)EMACSwizzleData((uint32)curr_bd->next);
    q = q->next;
    }


  /* Indicate the start and end of the packet */
  bd_end->next = NULL;
  bd_end->flags_pktlen |= EMACSwizzleData(EMAC_BUF_DESC_EOP);

  txch->free_head = curr_bd;

  /* For the first time, write the HDP with the filled bd */
  if(txch->active_tail == NULL) {
    EMACTxHdrDescPtrWrite(hdkif->emac_base, (uint32)(active_head), (uint32)EMAC_CHANNELNUMBER);
  }

  /*
   * Chain the bd's. If the DMA engine, already reached the end of the chain,
   * the EOQ will be set. In that case, the HDP shall be written again.
   */
  else {			

    curr_bd = txch->active_tail;
    /* Wait for the EOQ bit is set */
    while (EMAC_BUF_DESC_EOQ != (EMACSwizzleData(curr_bd->flags_pktlen) & EMAC_BUF_DESC_EOQ))
    {
    }
    /* Don't write to TXHDP0 until it turns to zero */
	rtems_event_set events;
				rtems_bsdnet_event_receive (INTERRUPT_EVENT,
						RTEMS_WAIT|RTEMS_EVENT_ANY,
						RTEMS_NO_TIMEOUT,
						&events);
    while (((uint32)0U != *((uint32 *)0xFCF78600U)))
    {
    }
    curr_bd->next = (emac_tx_bd_t *)EMACSwizzleData((uint32)active_head);
    if (EMAC_BUF_DESC_EOQ == (EMACSwizzleData(curr_bd->flags_pktlen) & EMAC_BUF_DESC_EOQ)) {
      /* Write the Header Descriptor Pointer and start DMA */
      EMACTxHdrDescPtrWrite(hdkif->emac_base, (uint32)(active_head), (uint32)EMAC_CHANNELNUMBER);
    }
  }
   
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
  EMACTxIntHandler(hdkif);
  EMACCoreIntAck(EMAC_0_BASE, (uint32)EMAC_INT_CORE0_TX); 
  rtems_bsdnet_event_send (tms[0].txDaemonTid, INTERRUPT_EVENT);
  // tries=2;
  //  printf("f");           
}
static rtems_isr
tms_misc_interrupt_handler (rtems_vector_number v)
{
  rtems_interrupt_level pval;
  rtems_interrupt_disable (pval);


  EMACCoreIntAck(EMAC_0_BASE, (uint32)TMS570_IRQ_EMAC_MISC); 

  rtems_interrupt_enable (pval);

}
static rtems_isr
tms_thr_interrupt_handler (rtems_vector_number v)
{
  rtems_interrupt_level pval;
  rtems_interrupt_disable (pval);

  EMACCoreIntAck(EMAC_0_BASE, (uint32)TMS570_IRQ_EMAC_THRESH); 


  rtems_interrupt_enable (pval);
}
static rtems_isr
tms_rx_interrupt_handler (rtems_vector_number v)
{
    tms[0].rxInterrupts++;
//    rtems_bsdnet_event_send (tms[0].rxDaemonTid, INTERRUPT_EVENT);
 
rxch_t *rxch_int;
volatile emac_rx_bd_t *curr_bd, *curr_tail, *last_bd;
int rxBds=1,mlen;
struct mbuf *m=NULL;
struct ifnet *ifp;
rtems_interrupt_level pval;
    hdkif_t *hdkif;
    hdkif = tms[0].hdkif;
    rxch_int = &(hdkif->rxchptr);
    ifp = &tms[0].arpcom.ac_if;

    rtems_interrupt_disable (pval);

	  MGETHDR (m, M_NOWAIT, MT_DATA);
            if(m == NULL)
                return;
//
//   	  MCLGET (m, M_NOWAIT);
//            if(!(m->m_flags & M_EXT))
          m->m_flags |= M_EXT;
	  m->m_pkthdr.rcvif = ifp;
rxBds=1;

//          data_b = mtod(m,uint8 *);
//          data_b = m->m_ext.ext_buf; 
	  m->m_data=rxDataBuf[rxBds]; // 1
//
//	  m->m_data =malloc(MAX_FRAME_LENGTH * sizeof (uint8 *), M_DEVBUF, M_NOWAIT);//rxDataBuf[1]; // 1
//	  tms[0].rxMbuf[rxBds] = m;
//	  if (++rxBds == MAX_RX_PBUF_ALLOC-1) {
//	       break;
//       }

  struct ether_header *eh;	
  curr_bd = rxch_int->active_head;

  last_bd = rxch_int->active_tail;
  
  while((EMACSwizzleData((uint32)curr_bd->flags_pktlen) & EMAC_BUF_DESC_SOP) == EMAC_BUF_DESC_SOP) {

    /* Start processing once the packet is loaded */

	  if((EMACSwizzleData((uint32)curr_bd->flags_pktlen) & EMAC_BUF_DESC_OWNER)
       != EMAC_BUF_DESC_OWNER) {
        if((EMACSwizzleData((uint32)curr_bd->flags_pktlen) & EMAC_BUF_DESC_EOP)== EMAC_BUF_DESC_EOP)
        {

      /* this bd chain will be freed after processing */

		  rxch_int->free_head = curr_bd;
      curr_bd->flags_pktlen = EMACSwizzleData((uint32)EMAC_BUF_DESC_OWNER);	//02032019

      curr_bd->bufoff_len = EMACSwizzleData((uint32)MAX_TRANSFER_UNIT);	//02032019

      last_bd = curr_bd;	//02032019

      curr_bd = (emac_rx_bd_t *)EMACSwizzleData((uint32)curr_bd->next);	//02032019
     
//      m = tms[0].rxMbuf[rxBds];	//02032019
	m->m_len=EMACSwizzleData((uint32)last_bd->bufoff_len & 0xFFFF0000);//m->m_pkthdr.len=MAX_TRANSFER_UNIT-sizeof(struct ether_header);		//02032019
      memcpy(mtod(m,uint8 *),(uint8 *)EMACSwizzleData((uint32)last_bd->bufptr),m->m_len);	//02032019
//      memcpy(data_b,(void *)EMACSwizzleData((uint32)last_bd->bufptr),MAX_TRANSFER_UNIT);	//02032019

	eh = mtod (m, struct ether_header *);						//02032019
        m->m_len -= sizeof(struct ether_header);
//	m->m_len=m->m_pkthdr.len=MAX_TRANSFER_UNIT-sizeof(struct ether_header);		//02032019
	m->m_pkthdr.len=m->m_len;//,EMACSwizzleData((uint32)last_bd->flags_pktlen & 0xFFFF0000));		//02032019
	m->m_data+=sizeof(struct ether_header);						//02032019

	ether_input (ifp, eh, m);					//02032019

	EMACRxCPWrite(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER, (uint32)last_bd);             	//02032019
	    

    }

       
        else {
            while((EMACSwizzleData((uint32)curr_bd->flags_pktlen) & EMAC_BUF_DESC_EOP)!= EMAC_BUF_DESC_EOP)
            {
                MGETHDR (m, M_NOWAIT, MT_DATA);
                if(m == NULL)
                  return;
                m->m_flags |= M_EXT;
                m->m_pkthdr.rcvif = ifp;
                m->m_data=rxDataBuf[++rxBds]; // 1
                curr_bd->flags_pktlen = EMACSwizzleData((uint32)EMAC_BUF_DESC_OWNER);
            
                curr_bd->bufoff_len = EMACSwizzleData((uint32)MAX_TRANSFER_UNIT);
            
                last_bd = curr_bd;
            
                curr_bd = (emac_rx_bd_t *)EMACSwizzleData(curr_bd->next);
    
    	    m->m_len=EMACSwizzleData((uint32)last_bd->bufoff_len & 0xFFFF0000);
  
  //	    m_append(m, mlen, cp);
              /**Append len bytes of data cp to the mbuf chain.  Extend the mbuf
                 	   chain if the	new data does not fit in existing space.	**/
  
                memcpy(mtod(m,uint8 *),(uint8 *)EMACSwizzleData((uint32)last_bd->bufptr),m->m_len);	//02032019
    	    eh = mtod (m, struct ether_header *);						//02032019
                m->m_len -= sizeof(struct ether_header);
    	    m->m_data+=sizeof(struct ether_header);
    	    ether_input (ifp, eh, m);
            }
          /* Acknowledge that this packet is processed */
	EMACRxCPWrite(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER, (uint32)last_bd);             	//02032019
        }
      rxch_int->active_head = curr_bd;                                              	
      curr_tail = rxch_int->active_tail;                                                
      curr_tail->next = (emac_rx_bd_t *)EMACSwizzleData((uint32)rxch_int->free_head);   
      last_bd->next = NULL;                                                             



        /**
         * Check if the reception has ended. If the EOQ flag is set, the NULL
         * Pointer is taken by the DMA engine. So we need to write the RX HDP
         * with the next descriptor.
         */

        if((EMACSwizzleData((uint32)curr_tail->flags_pktlen) & EMAC_BUF_DESC_EOQ) == EMAC_BUF_DESC_EOQ) {

          EMACRxHdrDescPtrWrite(hdkif->emac_base, (uint32)(rxch_int->free_head), (uint32)EMAC_CHANNELNUMBER);
        }

  
        rxch_int->free_head  = curr_bd;
        rxch_int->active_tail = last_bd;
		
       
    }



  }
        EMACCoreIntAck(EMAC_0_BASE, (uint32)EMAC_INT_CORE0_RX);
        EMACCoreIntAck(EMAC_0_BASE, (uint32)EMAC_INT_CORE0_TX);
    rtems_interrupt_enable (pval);

}



/*
 * SCC reader task
 */
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

  rtems_interrupt_level pval;
  


hdkif=sc->hdkif;
rxch_int = &(sc->hdkif->rxchptr);
/*
	 * Allocate space for incoming packets and start reception
*/
for (rxBds = 0 ; ;) {
	  MGETHDR (m, M_WAIT, MT_DATA);
   	  MCLGET (m, M_WAIT);
	  m->m_pkthdr.rcvif = ifp;
	  m->m_data=rxDataBuf[1]; // 1
	  sc->rxMbuf[rxBds] = m;
	  if (++rxBds == MAX_RX_PBUF_ALLOC-1) {
	       break;
       }
}
rxBds=1;
 
	//	sc->rxMbuf[rxBds]->m_data=rxch_int->active_head->bufptr;
	/*
	 * Input packet handling loop
	 */
//for (;;) {
//                
//		/*
//		 * Wait for packet if there's not one ready
//		 */
//
//		if ((*(uint32_t *)(0xFCF780A4))== 0x00000000) {
//			while ((*(uint32_t *)(0xFCF780A4)) == 0x00000000 ) {
//      	    	     	rtems_event_set events;
//     //	    		rtems_interrupt_level level;
//			rtems_interrupt_disable (pval);
////			rtems_interrupt_enable (level);
//
//				/*
//				 * Unmask RXF (Full frame received) event
//				 */
//			
//				rtems_bsdnet_event_receive (INTERRUPT_EVENT,
//						RTEMS_WAIT|RTEMS_EVENT_ANY,
//						RTEMS_NO_TIMEOUT,
//						&events);
//				
//			}
//		}
//
//			struct ether_header *eh;	
//
//
//  /* Get the buffer descriptors which contain the earliest filled data */
//
// // rtems_interrupt_disable(pval);
//  curr_bd = rxch_int->active_head;
//
//  last_bd = rxch_int->active_tail;
//
//  /**
//   * Process the descriptors as long as data is available
//   * when the DMA is receiving data, SOP flag will be set
//  */
//
//  while((EMACSwizzleData((uint32)curr_bd->flags_pktlen) & EMAC_BUF_DESC_SOP) == EMAC_BUF_DESC_SOP) {
//
//    /* Start processing once the packet is loaded */
//
//	  if((EMACSwizzleData((uint32)curr_bd->flags_pktlen) & EMAC_BUF_DESC_OWNER)
//       != EMAC_BUF_DESC_OWNER) {
//
//      /* this bd chain will be freed after processing */
//
//		  rxch_int->free_head = curr_bd;
//
//      /* Get the total length of the packet. curr_bd points to the start
//       * of the packet.
//       */
//
//      /* 
//       * The loop runs till it reaches the end of the packet.
//       */
////02032019     curr_bd->flags_pktlen = EMACSwizzleData((uint32)EMAC_BUF_DESC_OWNER);	
//
////02032019     curr_bd->bufoff_len = EMACSwizzleData((uint32)MAX_TRANSFER_UNIT);	
//
////02032019     last_bd = curr_bd;	
//
////02032019     curr_bd = (emac_rx_bd_t *)EMACSwizzleData((uint32)curr_bd->next);	
//	      
////02032019      m = sc->rxMbuf[rxBds];
////
////04032019      memcpy(mtod(m,void *),(void *)EMACSwizzleData((uint32)last_bd->bufptr),MAX_TRANSFER_UNIT);	
////04032019	EMACRxCPWrite(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER, (uint32)last_bd);             	
////04032019
////04032019	eh = mtod (m, struct ether_header *);						
////04032019	m->m_len=m->m_pkthdr.len=MAX_TRANSFER_UNIT-sizeof(struct ether_header);		
////04032019	m->m_data+=sizeof(struct ether_header);						
////04032019
////04032019	ether_input (ifp, eh, m);					
////04032019	MGETHDR (m, M_WAIT, MT_DATA);                                   
////04032019	MCLGET (m, M_WAIT);                                             
////04032019	m->m_pkthdr.rcvif = ifp;      
////04032019	m->m_data=rxDataBuf[rxBds];   
////04032019   	sc->rxMbuf[rxBds] = m;        
////
////02032019    while((EMACSwizzleData((uint32)curr_bd->flags_pktlen) & EMAC_BUF_DESC_EOP)!= EMAC_BUF_DESC_EOP)
////02032019      {
////02032019        curr_bd->flags_pktlen = EMACSwizzleData((uint32)EMAC_BUF_DESC_OWNER);
////02032019        
////02032019        curr_bd->bufoff_len = EMACSwizzleData((uint32)MAX_TRANSFER_UNIT);
////02032019        
////02032019        last_bd = curr_bd;
////02032019        
////02032019        curr_bd = (emac_rx_bd_t *)EMACSwizzleData(curr_bd->next);
////
////02032019	  m_append(m, len, cp)
////02032019               /**Append len bytes of data cp to the mbuf chain.  Extend the mbuf
////02032019               	   chain if the	new data does not fit in existing space.	**/
////
////02032019	m->m_len=m->m_pkthdr.len=MAX_TRANSFER_UNIT-sizeof(struct ether_header);
////02032019	m->m_data+=sizeof(struct ether_header);
////02032019	ether_input (ifp, eh, m);
////
////
////        m = sc->rxMbuf[rxBds];//------------------
////        m->m_data=last_bd->bufptr;
////       	eh = mtod (m, struct ether_header *);
////
////	MGETHDR (m, M_WAIT, MT_DATA);
////	MCLGET (m, M_WAIT);
////	m->m_pkthdr.rcvif = ifp;     
////      	sc->rxMbuf[rxBds] = m;	//---------------------------------------------------------
//// 	if(++rxBds == MAX_RX_PBUF_ALLOC-1)
////	  rxBds=0;
////		      
////02032019     }
//      
//
//      curr_bd->flags_pktlen = EMACSwizzleData((uint32)EMAC_BUF_DESC_OWNER);	//02032019
//
//      curr_bd->bufoff_len = EMACSwizzleData((uint32)MAX_TRANSFER_UNIT);	//02032019
//
//      last_bd = curr_bd;	//02032019
//
//      curr_bd = (emac_rx_bd_t *)EMACSwizzleData((uint32)curr_bd->next);	//02032019
//     
//      //    EMACRxCPWrite(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER, (uint32)last_bd);
//
//      m = sc->rxMbuf[rxBds];	//02032019
//
////            while((EMACSwizzleData((uint32)last_bd->flags_pktlen) & EMAC_BUF_DESC_OWNER)==EMAC_BUF_DESC_OWNER)
////      	{
////	  //wait for it to be ready
////      	}
////        m->m_data=last_bd->bufptr;
////	eh = mtod (m, struct ether_header *);
////	m->m_len=m->m_pkthdr.len=MAX_TRANSFER_UNIT-sizeof(struct ether_header);//last_bd->flags_pktlen&(0x0000FFFF)-sizeof(struct ether_header);//MAX_TRANSFER_UNIT-sizeof(struct ether_header);
////	m->m_data+=sizeof(struct ether_header);
////	ether_input (ifp, eh, m);
//
//        memcpy(mtod(m,void *),(void *)EMACSwizzleData((uint32)last_bd->bufptr),EMACSwizzleData((uint32)last_bd->len));//MAX_TRANSFER_UNIT);	//02032019
//	EMACRxCPWrite(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER, (uint32)last_bd);             	//02032019
//
//	eh = mtod (m, struct ether_header *);						//02032019
//	m->m_len=m->m_pkthdr.len=MAX_TRANSFER_UNIT-sizeof(struct ether_header);		//02032019
//	m->m_data+=sizeof(struct ether_header);						//02032019
//
//	ether_input (ifp, eh, m);					//02032019
//	MGETHDR (m, M_WAIT, MT_DATA);                                   //02032019
//	MCLGET (m, M_WAIT);                                             //02032019
//	m->m_pkthdr.rcvif = ifp;      //02032019
//	m->m_data=rxDataBuf[rxBds];   //02032019
//   	sc->rxMbuf[rxBds] = m;        //02032019
//
//   	if(++rxBds == MAX_RX_PBUF_ALLOC-1)
//			rxBds=0;
//	    
//      rxch_int->active_head = curr_bd;                                              	
//      curr_tail = rxch_int->active_tail;                                                
//      curr_tail->next = (emac_rx_bd_t *)EMACSwizzleData((uint32)rxch_int->free_head);   
//      last_bd->next = NULL;                                                             
//
//
//        /**
//         * Check if the reception has ended. If the EOQ flag is set, the NULL
//         * Pointer is taken by the DMA engine. So we need to write the RX HDP
//         * with the next descriptor.
//         */
//
//        if((EMACSwizzleData((uint32)curr_tail->flags_pktlen) & EMAC_BUF_DESC_EOQ) == EMAC_BUF_DESC_EOQ) {
//
//          EMACRxHdrDescPtrWrite(hdkif->emac_base, (uint32)(rxch_int->free_head), (uint32)EMAC_CHANNELNUMBER);
//        }
//
//  
//        rxch_int->free_head  = curr_bd;
//        rxch_int->active_tail = last_bd;
//		
//	/*	if(curr_bd == 0xfc5210b0){
//	  rxch_int->active_head=0xfc521010;
//	   EMACRxHdrDescPtrWrite(hdkif->emac_base, (uint32)rxch_int->active_head, (uint32)EMAC_CHANNELNUMBER);
//	}
//	*/
//       
//    }
//  }
//        EMACCoreIntAck(EMAC_0_BASE, (uint32)EMAC_INT_CORE0_RX);
//
//  rtems_interrupt_enable(pval);
//	
//}
}
//#if 0
static void
sendpacket (struct ifnet *ifp, struct mbuf *m)
{
  bool i=false;
  struct tms_softc *sc = ifp->if_softc;
  rtems_interrupt_level pval;
  

  rtems_interrupt_disable(pval);
    i=EMACTransmit_mbuf(sc->hdkif, m);
  rtems_interrupt_enable(pval);
    
}
//#endif
#if 0
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
  while(a){   
        myframe=mtod(a,uint8 *);
        if (a->m_len) {
	  memcpy(&test[0]+acc,myframe,a->m_len);
	  acc+=a->m_len;
	}
       MFREE (a, b);
       a=b;
  }
  ti_buffer[im].payload=&test[0];
  ti_buffer[im].next=NULL;
  if(acc<MIN_FRAME_LENGTH)
    ti_buffer[im].tot_len=ti_buffer[im].len=65;
  else
    ti_buffer[im].tot_len=ti_buffer[im].len=acc;


    i=EMACTransmit(sc->hdkif, &ti_buffer[0]);
    //    memset(test,0,1514);
    //  if(i==false)
    //  printf("Could not send frame!");
    // else
    //   printf("good");
    //
    //
  uint8 *myframe;
  uint8 test[1514];
  struct pbuf_struct ti_buffer[24];
  struct mbuf *a,*b;
  bool i=false;
  int im=0,diff_min=0,acc=0;
  struct tms_softc *sc = ifp->if_softc;
    // 
  a=m;
  if(a->m_ext.ext_buf!=a->m_data && (a->m_flags&M_EXT)==M_EXT)
     a->m_data=a->m_ext.ext_buf;
  ti_buffer[im].tot_len=a->m_pkthdr.len;
  while(a){ 
    myframe=mtod(a,uint8 *);
    if(a->m_len == 0)
      {
	// myframe=mtod(a,uint8 *);
	//	memset(test,1,1514);
       
      }
    else if(a->m_len<MIN_FRAME_LENGTH && acc<MIN_FRAME_LENGTH)
      {
	diff_min=MIN_FRAME_LENGTH-a->m_len;
	memcpy(&test[0]+acc,myframe,a->m_len+diff_min);	
	//		printf("%d\n",);
	//	ti_buffer[im].payload=myframe;
	//	ti_buffer[im].tot_len=a->m_pkthdr.len+acc ;
	//	ti_buffer[im].len=MIN_FRAME_LENGTH;    	
	acc+=a->m_len;
      }
    else if(a->m_len+acc > MAX_FRAME_LENGTH && acc>0)
      {
	ti_buffer[im].payload=&test[0];
	ti_buffer[im].len=acc+diff_min;
	ti_buffer[im].next=&ti_buffer[im+1];
	ti_buffer[im].tot_len+=diff_min;
	im++;
	memcpy(&test[0],myframe,a->m_len);
	ti_buffer[im].payload=&test[0];
	ti_buffer[im].len=a->m_len;   
	ti_buffer[im].next=&ti_buffer[im+1];   
	im++;
	diff_min=0;
	acc=0;
      }
    else if(a->m_len > MAX_FRAME_LENGTH )
      {
	memcpy(&test[0],myframe,1500U);
	ti_buffer[im].payload=&test[0];
	ti_buffer[im].len=1500;   
	ti_buffer[im].next=&ti_buffer[im+1];   
	im++;
	acc=0;
	diff_min=0;
      }
    else{
        memcpy(&test[0]+acc,myframe,a->m_len);
	ti_buffer[im].payload=&test[0];
	ti_buffer[im].len=a->m_len+acc;   
	ti_buffer[im].next=&ti_buffer[im+1];   
	im++;
	acc=0;
	diff_min=0;

    }
       MFREE (a, b);
       a=b;
       // a=a->m_next;
    
    //    a=a->m_next; //a se queda igual
     // memcpy(&test[0],myframe,a->m_len);
  }

  if(diff_min!=0){
    ti_buffer[im].payload=&test[0];
    ti_buffer[im].len=acc+diff_min;
    ti_buffer[im].next=NULL;
    ti_buffer[im].tot_len+=diff_min;
  }
  else{
    ti_buffer[im-1].next=NULL;
  }
  /*
for(j=0;j<im;j++)
  ti_buffer[j].tot_len=acc;
  */

 


    i=EMACTransmit(sc->hdkif, &ti_buffer[0]);
    //    memset(test,0,1514);
    //  if(i==false)
    //  printf("Could not send frame!");
    // else
    //   printf("good");

}
#endif
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
  for(temp = 0U; temp < EMAC_HWADDR_LEN; temp++) {
    hdkif->mac_addr[temp] = macaddr[(EMAC_HWADDR_LEN - 1U) - temp];
  }
  /*Initialize the EMAC, EMAC Control and MDIO modules. */
  EMACInit(hdkif->emac_ctrl_base, hdkif->emac_base);
  MDIOInit(hdkif->mdio_base, MDIO_FREQ_INPUT, MDIO_FREQ_OUTPUT);
  
  /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
  while(delay != 0U)
  {
  /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
    delay--;
  }
  

  /* Enable Broadcast if enabled in the GUI. */
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
#if(EMAC_BROADCAST_ENABLE)
      EMACRxBroadCastEnable(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER);
#else
   /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */
   /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */
      EMACRxBroadCastDisable(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER);
#endif

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
//#ifdef ULAN
//#else
  EMACDMAInit(hdkif);
//#endif
  /* Acknowledge receive and transmit interrupts for proper interrupt pulsing*/
  EMACCoreIntAck(hdkif->emac_base, (uint32)EMAC_INT_CORE0_RX);
  EMACCoreIntAck(hdkif->emac_base, (uint32)EMAC_INT_CORE0_TX);

#if(EMAC_UNICAST_ENABLE)
      EMACRxUnicastSet(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER);
#else
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */
      EMACRxUnicastClear(hdkif->emac_base, (uint32)EMAC_CHANNELNUMBER);
#endif

#if(EMAC_TX_ENABLE)
      EMACTxEnable(hdkif->emac_base);
      EMACTxIntPulseEnable(hdkif->emac_base, hdkif->emac_ctrl_base, (uint32)EMAC_CHANNELNUMBER, (uint32)EMAC_CHANNELNUMBER);
#else
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */
      EMACTxDisable(hdkif->emac_base);
      EMACTxIntPulseDisable(hdkif->emac_base, hdkif->emac_ctrl_base, (uint32)EMAC_CHANNELNUMBER, (uint32)EMAC_CHANNELNUMBER);     
#endif
      rxch = &(hdkif->rxchptr);

  /* Enable Receive and Receive Interrupt. Then start receiving by writing to the HDP register. */
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
#if(EMAC_RX_ENABLE)
      EMACNumFreeBufSet(hdkif->emac_base,(uint32)EMAC_CHANNELNUMBER , (uint32)MAX_RX_PBUF_ALLOC);
      EMACRxEnable(hdkif->emac_base);
      EMACRxIntPulseEnable(hdkif->emac_base, hdkif->emac_ctrl_base, (uint32)EMAC_CHANNELNUMBER, (uint32)EMAC_CHANNELNUMBER);
      /* Write to the RX HDP for channel 0 */
      /*SAFETYMCUSW 45 D MR:21.1 <APPROVED> "Valid non NULL input parameters are assigned in this driver" */     
      EMACRxHdrDescPtrWrite(hdkif->emac_base, (uint32)rxch->active_head, (uint32)EMAC_CHANNELNUMBER);
#else
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */    
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */  
      EMACRxDisable(hdkif->emac_base);
      EMACRxIntPulseDisable(hdkif->emac_base, hdkif->emac_ctrl_base, (uint32)EMAC_CHANNELNUMBER, (uint32)EMAC_CHANNELNUMBER);     
#endif
  /* Enable MII if enabled in the GUI. */
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
#if(EMAC_MII_ENABLE)
      EMACMIIEnable(hdkif->emac_base);
#else
      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */  
      EMACMIIDisable(hdkif->emac_base);
#endif
  EMACRMIISpeedSet(hdkif->emac_base,EMAC_RMIISPEED_100MBPS);


  
//#if (EMAC_FULL_DUPLEX_ENABLE)
//        EMACDuplexSet(EMAC_0_BASE, (uint32)EMAC_DUPLEX_FULL);
//#else
//        /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition arameter is taken as input from GUI." */
//        EMACDuplexSet(EMAC_0_BASE, (uint32)EMAC_DUPLEX_HALF);
//#endif
//
//  /* Enable Loopback based on GUI Input */ 
//  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
//#if(EMAC_LOOPBACK_ENABLE)
//      EMACEnableLoopback(hdkif->emac_base);
//#else
//      /*SAFETYMCUSW 1 J MR:14.1 <APPROVED> "If condition parameter is taken as input from GUI." */  
//      EMACDisableLoopback(hdkif->emac_base);
//#endif

  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Parameter is taken as input from GUI." */
//      *(uint32_t *)(0xFCF78100)|=0x00800000;//0x00A00000;
      //      *(uint32_t *)(0xFCF78100)&=0xFFFFDFFF;//0x00A00000;
      //
//	_enable_interrupt_();	
//	asm volatile (" cpsie	if");
//	asm volatile (	"bx	lr");

      if(retVal==EMAC_ERR_CONNECT)
	printf("emachwinit failed...");  
    status = bsp_interrupt_vector_enable(
					  TMS570_IRQ_EMAC_TX
					   );

      status = rtems_interrupt_handler_install( TMS570_IRQ_EMAC_TX,
      "tms1",
      RTEMS_INTERRUPT_SHARED,
      tms_tx_interrupt_handler,
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
      tms_rx_interrupt_handler,
      NULL
						);

if (status != RTEMS_SUCCESSFUL) 
      printf ("Can't install int rx handler: `%s'\n", rtems_status_text (status));

      status = bsp_interrupt_vector_enable(
					 TMS570_IRQ_EMAC_MISC 
					   );

      status = rtems_interrupt_handler_install( TMS570_IRQ_EMAC_MISC,
      "tms1",
      RTEMS_INTERRUPT_SHARED,
      tms_misc_interrupt_handler,
      NULL
						);

if (status != RTEMS_SUCCESSFUL) 
      printf ("Can't install int misc handler: `%s'\n", rtems_status_text (status));



      status = bsp_interrupt_vector_enable(
					 TMS570_IRQ_EMAC_THRESH 
					   );

      status = rtems_interrupt_handler_install( TMS570_IRQ_EMAC_THRESH,
      "tms1",
      RTEMS_INTERRUPT_SHARED,
      tms_thr_interrupt_handler,
      NULL
						);

if (status != RTEMS_SUCCESSFUL) 
      printf ("Can't install int threshold handler: `%s'\n", rtems_status_text (status));


 tms[0].rxMbuf = malloc (tms[0].rxBdCount * sizeof *tms[0].rxMbuf, M_MBUF, M_NOWAIT);
}
/*
 * Initialize and start the device
 */
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
                sc->txDaemonTid = rtems_bsdnet_newproc ("TMStx", 4096, tms_txDaemon, sc);
//		sc->rxDaemonTid = rtems_bsdnet_newproc ("TMSrx", 4096, tms_rxDaemon, sc);

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

