/**
	****************************************************************************
	*	@file: 		ct_security_device										   											 *
	*	@author: 	��Ȩ���� �Ϻ���ͼ����Ƽ����޹�˾ 2014-2016								   										 *	
	*	@data:	 	01-March-2016											   													 *
	*	@Copyright (c) 2014--2016 Catosoft.Co.Ltd. All rights reserved.		   		 *
	*	��Ȩ���� �Ϻ���ͼ����Ƽ����޹�˾ 2014-2016 2014-2016						   							 *
	****************************************************************************
**/
#ifndef CT_SECURITY_DEVICE
#define CT_SECURITY_DEVICE

#include "Common.h"

#ifdef __cplusplus
	extern "C"
	{
#endif		

	/**
		@bried:  
				Generate the request for registration code.
		@param:	
				deviceID: 	Input buffer,deviceID is device unique device identifier,
							The length of deviceID should be 16 bytes.
				req:		Output buffer,req is the request code that should send it to cloud
							The length of req should be 16 bytes.
		@retval
				0:	Execution is success.
				-2:	The buffer that input is NULL pointer.
				
	**/
	extern int ct_gen_request (IN char * deviceID, OUT char* req);

	/**
		@bried:
				After registered, device check if it is registered from time to time.
		@param
				deviceID:	Input buffer,deviceID is device unique device identifier,
							The length of deviceID should be 16 bytes.
				regCode:	Input buffer,cloud generate register code and send it to device
							The length of regCode should be 32 bytes
		@retval
				0:	Execution is success
				-1:	Device do not register from cloud.
				-2:	The input buffer if NULL
	**/
	extern int ct_check_reg_status(IN char * deviceID, IN char* regCode);

	/**
		@bried:
				To encryption the buffer that send to cloud
		@param:
				regCode:	Input buffer,cloud generate register code and send it to device
							The length of regCode should be 32 bytes
				buf:		Input buffer,Device send the buffer to cloud
							The length of buf should be 16 bytes.
		@retval:
				0:	Execution is success
				-1:	Device do not register from cloud.
				-2:	The input buffer if NULL
	**/
	extern int ct_transform(IN char* regCode, IN OUT char* buf);
#ifdef __cplusplus
	}
#endif
 
#endif


 