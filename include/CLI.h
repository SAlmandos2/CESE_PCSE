/*
 * CLI.h
 *
 *  Created on: 16 abr. 2021
 *      Author: Santiago
 */

#ifndef CLI_H_
#define CLI_H_

/* Based on FreeRTOS_CLI interpreter */


/*====================[Typedefs enums and structs]==========================*/

/* The prototype to which callback functions used to process command line
commands must comply.  pcWriteBuffer is a buffer into which the output from
executing the command can be written, xWriteBufferLen is the length, in bytes of
the pcWriteBuffer buffer, and pcCommandString is the entire string as input by
the user (from which parameters can be extracted).*/
typedef BaseType_t (*pdCOMMAND_LINE_CALLBACK)( const char *pcCommandString, void *xUserData );

/*
 * @brief	The structure that defines command line commands.
 * 			A command line command should be defined by declaring a const structure of this type.
 */
typedef struct xCOMMAND_LINE_INPUT
{
	const char * const pcCommand;				/* The command that causes pxCommandInterpreter to be executed.  For example "help".  Must be all lower case. */
	const pdCOMMAND_LINE_CALLBACK pxCommandInterpreter;	/* A pointer to the callback function that will return the output generated by the command. */
	int8_t cExpectedNumberOfParameters;			/* Commands expect a fixed number of parameters, which may be zero. */
} CLI_Command_Definition_t;

/*
 * @brief	The structure that holds each element of CLI commands
 */
typedef struct xCOMMAND_INPUT_LIST
{
	const CLI_Command_Definition_t *pxCommandLineDefinition;
	struct xCOMMAND_INPUT_LIST *pxNext;
} CLI_Definition_List_Item_t;

/*
 * @brief	CLI handle structure
 */
typedef struct{
	CLI_Definition_List_Item_t* commandList;
	char* tokens;
} CLI_Definition_t;


/*====================[Functions definitions]===============================*/

/*
 * @brief	Register the command passed in using the pxCommandToRegister parameter.
 * 			Registering a command adds the command to the list of commands that are
 * 			handled by the command interpreter.  Once a command has been registered it
 * 			can be executed from the command line.
 * @param	cliCommandList: Handler of CLI commands and token to process
 * @param	pxCommandToRegister: pointer to structure that holds the defined command
 * @return	pdPASS if command registered, pdFAIL if something went wrong
 */
BaseType_t CLI_RegisterCommand( CLI_Definition_t *cliCommandList, const CLI_Command_Definition_t * const pxCommandToRegister );

/*
 * @brief	Runs the command interpreter for the command string "pcCommandInput".
 * 			Data can be stored or passed to command function with xUserData;
 *
 * @param	cliCommandList: Handler of CLI commands and token to process
 * @param	pcCommandInput: pointer to string to process
 * @param	xUserData: pointer to data to send or receive from user CLI function
 * @return	pdPASS if command found and processed correctly,
 * 			pdFAIL if command not found or pdFAIL returned from user function
 */
BaseType_t CLI_ProcessCommand(  CLI_Definition_t *cliCommandList, const char * const pcCommandInput, void * xUserData );

/*-----------------------------------------------------------*/

/*
 * @brief	Return a pointer to the uxWantedParameter'th word in pcCommandString.
 * @param	cliCommandList: Handler of CLI commands and token to process
 * @param	pcCommandInput: pointer to string to process
 * @paramu	xWantedParameter: index of wanted parameter
 * @param	pxParameterStringLength: pointer to receive wanted parameter length
 */
const char *CLI_GetParameter( CLI_Definition_t *cliCommandList, const char *pcCommandString, UBaseType_t uxWantedParameter, BaseType_t *pxParameterStringLength );

#endif /* CLI_H_ */
