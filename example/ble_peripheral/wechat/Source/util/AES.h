/**************************************************************************************************
 
  Shanghai QST Corporation confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Shanghai QST 
  Corporation ("QST"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of QST. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a QST Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  QST OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/


#ifndef __AES_H
#define __AES_H

#ifdef __cplusplus
	extern "C" {
#endif

// ??bit???????????????????????? 128??192 ?? 256 ????
#define AES_KEY_LENGTH	128

// ??????????
#define AES_MODE_ECB	0				// ??????????????????????????
#define AES_MODE_CBC	1				// ????????????????
#define AES_MODE		AES_MODE_CBC


///////////////////////////////////////////////////////////////////////////////
//	????????	AES_Init
//	??????		??????????????????????????????
//	??????????	pKey -- ?????????????????????? AES_KEY_LENGTH/8 ??????
//	??????????	????
//	????????	????
///////////////////////////////////////////////////////////////////////////////
void AES_Init(const void *pKey);

//////////////////////////////////////////////////////////////////////////
//	????????	AES_Encrypt
//	??????		????????
//	??????????	pPlainText	-- ??????????????????????????????nDataLen??????
//				nDataLen	-- ??????????????????????????????AES_KEY_LENGTH/8??????????
//				pIV			-- ????????????????????ECB????????????NULL??
//	??????????	pCipherText	-- ??????????????????????????????????pPlainText??????
//	????????	????
//////////////////////////////////////////////////////////////////////////
void  AES_Encrypt(const unsigned char *pPlainText, unsigned char *pCipherText, 
				 unsigned int nDataLen, const unsigned char *pIV);

//////////////////////////////////////////////////////////////////////////
//	????????	AES_Decrypt
//	??????		????????
//	??????????	pCipherText -- ??????????????????????????????nDataLen??????
//				nDataLen	-- ??????????????????????????????AES_KEY_LENGTH/8??????????
//				pIV			-- ????????????????????ECB????????????NULL??
//	??????????	pPlainText  -- ??????????????????????????????????pCipherText??????
//	????????	????
//////////////////////////////////////////////////////////////////////////
void AES_Decrypt(unsigned char *pPlainText, const unsigned char *pCipherText, 
				 unsigned int nDataLen, const unsigned char *pIV);

//????????????????????????????1??????????0
unsigned char app_data_encode_aes(char *input, char *output, unsigned short *slen);
unsigned char app_data_decode_aes(unsigned char *input, char *output, unsigned short *slen);
unsigned int AES_Encrypt_PKCS7(const unsigned char *pPlainText, unsigned char *pCipherText, 
				 unsigned int nDataLen, const unsigned char *pIV);
unsigned int AES_get_length(unsigned int length);
void AES_free(unsigned char* p);
#ifdef __cplusplus
	}
#endif


#endif	// __AES_H
