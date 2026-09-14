#include "Parsing.h"
#include "FSM.h"
#include <Encoder.h>
#include <HelperFunctions.h>

String inputBuffer = "";
GcodeToken TokenArray[MAX_TOKENS];

void initialiseTokenArray(void) {
    TokenArray[G].Set("G0");
    TokenArray[X].Set("X0");
    TokenArray[Y].Set("Y0");
    TokenArray[F_token].Set("F300");
}


/* reads line from arduino serial buffer into input buffer */
int readLine(void) {

    int flag = 1;   // set to 0 when tokensise gets called
    int validCommand = 2;   // 2 = incomplete command, 1 = invalid command, 0 = successful command

    while(Serial.available() && flag) {  // Serial.available() returns how many characters are waiting to be read in the arduinos serial buffer 

        char c = Serial.read();  // pulls the first (character) from the buffer

        if (c == '\n') {
            flag = 0;
            validCommand = tokenise(inputBuffer); // when new line character is reahced send input buffer to parseGcode function
            inputBuffer = "";   // reset buffer
        } else {
            inputBuffer += c;
        }
    }
    return (validCommand) ;    // will be 1 (error) if command reads complete line and command invalid
}


int tokenise(String line) {

    line = tidyString(line);   // remove spaces, make uppercase, and remove comment
    int err = letterSignCheck(line);

    if (err != 0) {
        Serial.print("Invalid Command: ");

        switch (err) 
        {
        case 1:
            Serial.println("Too Many Letters");
            break;
        case 2:
            Serial.println("Same Letter Repeated");
            break;
        case 4:
            Serial.println("No G or M command");
            break;
        case 5:
            Serial.println("Contains non-AlphaNumeric Character/s");
            break;
        case 6:
            Serial.println("Invalid Negative Sign/s ");
            break;
        case 7:
            Serial.println("Internal Logic Error");
            break;
        case 8:
            Serial.println("Too Many Negative Signs");
            break;
        case 9:
            Serial.println("Contains Letters Which Are Not G,M,X,Y or F");
            break;
        case 10:
            Serial.println("Contains G And M Commands");
            break;
        }
        return 1;   // all cases of errors return 1 to readline
    }

    

    String token = returnToken(line, 'M');

    if (FSM_getCurrentState() == STATE_FAULT) {

        if (token == "M999") {
            if ((line.length() == 4)) {   // State is Fault and command is only M999 (i.e. Correct Command)

                TokenArray[M].Set(token);
                return 0;

            } else {   // error: in fault state but command contains more than just M999

                Serial.println("Invalid Command: Must Contain Only M999 To Exit Error State");
                return 1;

            }
        } else {    // value after M is not 999

            Serial.println("Invalid Command: System Is In Fault State. Expected M999 Command");
            return 1;
        }
    } else {
        if (token == "M999") {

            Serial.println("Invalid Command: System Is Not In Fault State");
            return 1;

        } else if (token == "ERROR") {

            Serial.println("Invalid Command: System Is Not In Fault State. (Invalid M Command Regardless) ");
            return 1;

        }

        // token == "NoToken", no M token therefore, G01 or G28
        token = returnToken(line, 'G');

        if (token == "G28") {
            if ((line.length() == 3)) {   // command only contains G28, no X or Y or F (i.e. Correct Command)

                TokenArray[G].Set(token);
                return 0;

            } else {   // error: in fault state but command contains more than just M999

                Serial.println("Invalid Command: Must Contain Only G28 To Enter Homing State");
                return 1;

            }
        } else if (token == "G01" || token == "G1") {

            String Xtoken = returnToken(line, 'X');
            String Ytoken = returnToken(line, 'Y');
            String Ftoken = returnToken(line, 'F');

            if (Xtoken == "ERROR" && Ytoken == "ERROR") {
                Serial.println("Invalid Command: X and Y Distance");
                return 1;
            } else if (Xtoken == "ERROR") {
                Serial.println("Invalid Command: No X Distance");
                return 1;
            } else if (Ytoken == "ERROR") {
                Serial.println("Invalid Command: No Y Distance");
                return 1;
            } else if (Ftoken == "ERROR") {
                Serial.println("Invalid Command: Invalid Speed Value");
                return 1;
            }

            if (line.indexOf("X") == -1 && line.indexOf("Y") == -1) {   // does not contain X or Y

                Serial.println("Invalid Command: G01 Command Does Not Contain An X Or Y Distance");
                return 1;

            } else if (line.indexOf("X") != -1 && line.indexOf("Y") != -1) {    // Contains both X and Y command
                if (checkBounds(Xtoken) && checkBounds(Ytoken)) {
                    /* X and Y out of bounds */
                    Serial.println("Invalid Command: X And Y Distances Goes Out Of Bounds");
                    return 1;

                } else if (checkBounds(Xtoken)) {
                    /* X out of bounds */
                    Serial.println("Invalid Command: X Distance Goes Out Of Bounds");
                    return 1;

                } else if (checkBounds(Ytoken)) {
                    /* Y out of bounds */
                    Serial.println("Invalid Command: Y Distance Goes Out Of Bounds");
                    return 1;

                } else {
                    /* Both X and Y in bounds */
                    TokenArray[G].Set(token);
                    TokenArray[X].Set(Xtoken);
                    TokenArray[Y].Set(Ytoken);
                    if (Ftoken != "NoToken") { 
                        TokenArray[F_token].Set(Ftoken);        // Setter in GcodeToken throttles Speed
                    }

                    return 0;

                }
            } else if (line.indexOf("X") != -1) {   // Contains only X movement
                if (checkBounds(Xtoken)) {
                    Serial.println("Invalid Command: X Distance Goes Out Of Bounds");
                    return 1;

                } else {
                    TokenArray[G].Set(token);
                    TokenArray[X].Set(Xtoken);
                    if (Ftoken != "NoToken") { 
                        TokenArray[F_token].Set(Ftoken);        // Setter in GcodeToken throttles Speed
                    }
                    return 0;

                }

            } else if (line.indexOf("Y") != -1) {   // contains only Y movement
                if(checkBounds(Ytoken)) {
                    Serial.println("Invalid Command: Y Distance Goes Out Of Bounds");
                    return 1;

                } else {
                    TokenArray[G].Set(token);
                    TokenArray[Y].Set(Ytoken);
                    if (Ftoken != "NoToken") { 
                        TokenArray[F_token].Set(Ftoken);        // Setter in GcodeToken throttles Speed
                    }
                    return 0;
                }
            }
        } else if (token == "ERROR") {
            Serial.println("Invalid Command: Expected G01 Or G28.");
            return 1;
        } else {
            Serial.println("Invalid Command: Expected G01 Or G28.");
            return 1;
        }

        return 1;

    }

}


String returnToken(String line, char Letter) {

    int letterPos = line.indexOf(Letter);   // returns -1 if letter is not found
    int tokenEnd = letterPos;

    if (letterPos == -1) {
        return "NoToken";   // returns no token
    }

    while (tokenEnd+1 < (int)line.length() && (isDigit(line[tokenEnd+1]) || (line[tokenEnd+1] == '-')) ) {  // runs while next char is a digit or -ve sign
        tokenEnd++;
    }

    if (tokenEnd == letterPos) {
        // Error (No value after letter)
        Serial.println("Error 1");
        return "ERROR";
    }

    String token = line.substring(letterPos, (tokenEnd + 1));   // substring excludes end, hence (tokenEnd + 1)

    if ((Letter == 'M' || Letter == 'G' || Letter == 'F') && token.substring(1).toDouble() <= 0) {
        Serial.println("Error 2");
        return "ERROR";
    }
    return token;

}


String tidyString(String line) {

    int commaPos = line.indexOf(";");

    if (commaPos != -1) {
        line = line.substring(0, commaPos); // removes the comment from the line
    }

    line.trim();
    line.toUpperCase();  // any lowercase letters taken to upper
    line.replace(" ", "");  // removes spaces between tokens
    return line;

}


GcodeToken Parsing_getToken(int index) {

    return TokenArray[index];

}


void resetTokenArray() {
    TokenArray[G].Set("G0");
    TokenArray[X].Set("X0");
    TokenArray[Y].Set("Y0");
    // F token remains as it was in the previous intruction.
}

int letterSignCheck(String line) {   // pass tidied string into it to check there there is at most 4 letters and 2 -ve signs
    /*
    checks if there is at most 4 letters and only one of each (G/M, X, Y, F)
    checks that there are only 2 -ve signs

    Error Messages:
        0 = No Error (successful code)
        1 = too many letters
        2 = same letter repeated
        3 = no X and Y                  ------------(not in here)********************************
        4 = no G or M command
        5 = contains non alphanumeric character
        6 = invalid -ve sign/s
        7 = line of 0 length
        8 = too many -ve signs
        9 = contains letters that aren't G,M,X,Y, or F
       10 = contains G and M command
    */

    if (line.length() == 0) {
        return 7;   // line small (should never trigger due to previous code checks) // this is triggering
    }

    int totalG = 0;
    int totalM = 0;
    int totalX = 0;
    int totalY = 0;
    int totalF = 0;
    int totalLetters = 0;
    int numOfSigns = 0;

    for (unsigned int i = 0; i < line.length(); i++) {
        char c = line[i];

        switch(c) 
        {
        case 'G':
            totalG++;
            break;
        case 'M':
            totalM++;
            break;
        case 'X':
            totalX++;
            break;
        case 'Y':
            totalY++;
            break;
        case 'F':
            totalF++;
            break;
        case '-':
            numOfSigns++;
            break;
        }

        if (isAlpha(c)) {   // increment letter counter
            totalLetters++;
        } else if (!isAlphaNumeric(c) && c != '-') {    // if character is not alphaNumeric and not -ve sign, error.
            return 5;
        }
    }

    if (totalLetters > 4) {     // too many letters
        return 1;
    } else if (totalG > 1 || totalM > 1 || totalX > 1 || totalY > 1 || totalF > 1) {    // repeated letters
        return 2;
    } else if (totalG == 1 && totalM == 1) {    // G and M in one command
        return 10;
    } else if (totalG == 0 && totalM == 0) {    // No G or M in command
        return 4;
    } else if (totalLetters > (totalG + totalM + totalX + totalY + totalF)) {   // contains letters that aren't 
        return 9;
    } else if (numOfSigns > 2) {    // too many -ve signs
        return 8;
    } else if (numOfSigns == 0) {   // no -ve signs (valid)
        return 0;   
    }


    /* Need to get the indices of the -ve sign/s to see if they are valid */
    int indexOfSign[2] = {-1, -1};      // array for holding indices
    int j = 0;      // index starting pos

    for (unsigned int i = 0; i < line.length(); i++) {  // gets the indices of all -ve signs (at most 2, X-__ and Y-__)
        char c = line[i];
        if (c == '-') {
            indexOfSign[j] = i;
            j++;
        }
    }

    if (checkValidSign(indexOfSign, line)) { return 6; }     //error from signs not being valid
        
    return 0;   //letter and signs correct
    
}

int checkValidSign(int indices[], String line) {
    /* 
    0 = Valid
    1 = Error/Invalid
    */

    for (int i = 0; i < 2; i++) {
        int pos = indices[i];
        if (pos == -1) { break;  }
        
        if (pos == 0 || pos == line.length() - 1) {
            return 1; // error
        } else {
            if (!isAlpha(line[pos-1]) || !isDigit(line[pos+1])) { // letter before and number after
                return 1;   // error (either no letter before or no number after)
            }
        }
    }

    return 0; // 1 or both are valid -ve signs

}

bool checkBounds(String token) {
    long value = distanceToCounts(token.substring(1).toInt());
    //Serial.println(value);
    long currentLeft = Encoder_getLeftGlobalEncoderCount();
    long currentRight = Encoder_getRightGlobalEncoderCount();

    if (token[0] == 'X') {
        long currentX = (currentLeft + currentRight) / 2;
        if ((currentX + value) > X_MAX + BOUNDS_TOLERANCE || (currentX + value) < X_MIN - BOUNDS_TOLERANCE) {
            return 1;   // 1 = out of bounds
        }
    } else if (token[0] == 'Y') {
        long currentY = (currentLeft - currentRight) / 2;
        if ((currentY + value) > Y_MAX + BOUNDS_TOLERANCE || (currentY + value) < Y_MIN - BOUNDS_TOLERANCE) {
            return 1;   // 1 = out of bounds
        }
    }
    return 0;
}