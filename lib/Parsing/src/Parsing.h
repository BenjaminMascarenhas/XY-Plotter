#ifndef PARSE_H
#define PARSE_H

#include <Arduino.h>
#include <GcodeToken.h>

#define G 0
#define M 0
#define X 1
#define Y 2
#define F_token 3       // fix: #define F 3 collides with Arduino's F() macro used for storing strings

/* Functions */
void initialiseTokenArray(void);    // to be called in setup() in .ino
int readLine(void); // returns 0 if function succesfully reads a valid token/s, returns 1 if not
GcodeToken Parsing_getToken(int index);
void resetTokenArray();


/* Do not need to be in header file (only for internal use by Parsing.cpp) */
int tokenise(String Line);  // return 1 if token invalid. error messages printed by function.
String returnToken(String line, char Letter);
String tidyString(String line);
int letterSignCheck(String line);
int checkValidSign(int indices[], String line);

bool checkBounds(String token); // Returns 1 if out of bounds

#endif