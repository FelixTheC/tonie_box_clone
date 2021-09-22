#pragma once

#ifndef MFRC522_DELEGATE_H
#define MFRC522_DELEGATE_H

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <vector>
#include "MFRC522.h"

using std::string;


class MFRC522_Delegate
{
private:
    MFRC522 mfrc;
    MFRC522::MIFARE_Key key;
    MFRC522::StatusCode status;

    string card_uid;

    void log_mfrc_status(MFRC522 &mfrc522);
    void dump_string_to_byte_array(const string text, vector<byte> &array);

public:
    MFRC522_Delegate();
    bool readable_card_detected();
    void uid_bytes_to_string(MFRC522 &mfrc522);
    void write_to_rc(const string &text, const byte &start_block);

    void halt_picc();

    bool success;

    string get_card_uid();
    MFRC522 get_mfrc();
};

#endif // MFRC522_DELEGATE_H
