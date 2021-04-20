/*
 * CLI.c
 *
 *  Created on: 11 mar. 2021
 *      Author: Santiago-N
 */

/*====================[Includes]============================================*/
#include <string.h>
#include <stdint.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "CLI.h"


/*====================[Private Functions]===================================*/
/*
 * the number returned will be the number of parameters found.
 * from 1 to x
 *
 * to access that parameter use CLI_GetParameter(  , , returned - 1, )
 * */
static int8_t prvGetNumberOfParameters( CLI_Definition_t *cliCommandList, const char *pcCommandString )
{
	int8_t cParameters = 0;
	char* pcCommandStringAnalizis = NULL;

	do{
		pcCommandStringAnalizis = strpbrk( pcCommandString, cliCommandList->tokens );
		cParameters++;
	}while( pcCommandStringAnalizis != NULL );

	return cParameters;
}


/*====================[Functions definitions]===============================*/
BaseType_t CLI_RegisterCommand( CLI_Definition_t *cliCommandList, const CLI_Command_Definition_t * const pxCommandToRegister )
{
	CLI_Definition_List_Item_t *pxNewListItem;
	BaseType_t xReturn = pdFAIL;

	/* Check the parameter is not NULL. */
	configASSERT( pxCommandToRegister );

	/* Create a new list item that will reference the command being registered. */
	pxNewListItem = ( CLI_Definition_List_Item_t * ) pvPortMalloc( sizeof( CLI_Definition_List_Item_t ) );
	configASSERT( pxNewListItem );

	if( pxNewListItem != NULL )
	{
		/* Testing */
		/* Reference the command being registered from the newly created
		list item. */
		pxNewListItem->pxCommandLineDefinition = pxCommandToRegister;

		/* The new list item will get added to the end of the list, so
		pxNext has nowhere to point. */
		pxNewListItem->pxNext = NULL;

		if( cliCommandList->commandList == NULL)
		{
			taskENTER_CRITICAL();
			cliCommandList->commandList = pxNewListItem;
			taskEXIT_CRITICAL();
		}
		else
		{
			taskENTER_CRITICAL();
			/* Set the end of list marker to the new list item. */
			cliCommandList->commandList->pxNext = pxNewListItem;
			taskEXIT_CRITICAL();
		}

		xReturn = pdPASS;
	}

	return xReturn;
}


BaseType_t CLI_ProcessCommand(  CLI_Definition_t *cliCommandList, const char * const pcCommandInput, void * xUserData )
{
	const CLI_Definition_List_Item_t *pxCommand = NULL;
	BaseType_t xReturn = pdTRUE;
	const char *pcRegisteredCommandString;
	size_t xCommandStringLength;

	/* Search for the command string in the list of registered commands. */
	for( pxCommand = cliCommandList->commandList; pxCommand != NULL; pxCommand = pxCommand->pxNext )
	{
		pcRegisteredCommandString = pxCommand->pxCommandLineDefinition->pcCommand;
		xCommandStringLength = strlen( pcRegisteredCommandString );

		/* To ensure the string lengths match exactly, so as not to pick up
		a sub-string of a longer command, check the byte after the expected
		end of the string is either the end of the string or a space before
		a parameter. */
		if( strncmp( pcCommandInput, pcRegisteredCommandString, xCommandStringLength ) == 0 )
		{
			if ( ( pcCommandInput[ xCommandStringLength ] == *strpbrk( pcCommandInput, cliCommandList->tokens ) ) || ( pcCommandInput[ xCommandStringLength ] == 0x00 ) )
//				if( ( pcCommandInput[ xCommandStringLength ] == ' ' ) || ( pcCommandInput[ xCommandStringLength ] == 0x00 ) )
			{
				/* The command has been found.  Check it has the expected
				number of parameters.  If cExpectedNumberOfParameters is -1,
				then there could be a variable number of parameters and no
				check is made. */
				if( pxCommand->pxCommandLineDefinition->cExpectedNumberOfParameters >= 0 )
				{
					if( prvGetNumberOfParameters( cliCommandList, pcCommandInput ) != pxCommand->pxCommandLineDefinition->cExpectedNumberOfParameters )
					{
						xReturn = pdFALSE;
					}
				}

				break;
			}
		}
	}

	if( ( pxCommand != NULL ) && ( xReturn == pdFALSE ) )
	{
		/* The command was found, but the number of parameters with the command
		was incorrect. */
		pxCommand = NULL;
	}
	else if( pxCommand != NULL )
	{
		/* Call the callback function that is registered to this command. */
		xReturn = pxCommand->pxCommandLineDefinition->pxCommandInterpreter( pcCommandInput, xUserData );

		/* If xReturn is pdFALSE, then no further strings will be returned
		after this one, and	pxCommand can be reset to NULL ready to search
		for the next entered command. */
		if( xReturn == pdFALSE )
		{
			pxCommand = NULL;
		}
	}
	else
	{
		/* pxCommand was NULL, the command was not found. */
		xReturn = pdFALSE;
	}

	return xReturn;
}


const char *CLI_GetParameter( CLI_Definition_t *cliCommandList, const char *pcCommandString, UBaseType_t uxWantedParameter, BaseType_t *pxParameterStringLength )
{
	UBaseType_t uxParametersFound = 0;
	const char *pcReturn = NULL;
	const char *pcCommandStringAnalizis = pcCommandString;

	*pxParameterStringLength = 0;

	while( uxParametersFound < uxWantedParameter )
	{
		pcCommandStringAnalizis = strpbrk( pcCommandString, cliCommandList->tokens );
		uxParametersFound++;
		if( pcCommandStringAnalizis == NULL )
		   break;
		pcCommandString = pcCommandStringAnalizis + 1;
	}

	if( uxParametersFound == uxWantedParameter )
	{
		size_t len;
		pcCommandStringAnalizis = strpbrk( pcCommandString, cliCommandList->tokens );
		if( pcCommandStringAnalizis == NULL )
			len = strlen( pcCommandString );
		else
			len = pcCommandStringAnalizis - pcCommandString;

		pcReturn = pcCommandString;
		*pxParameterStringLength = len;
	}
	else
		pcReturn = NULL;

	return pcReturn;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

