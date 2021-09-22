#include "mfrc522_delegate.hpp"

MFRC522_Delegate::MFRC522_Delegate()
{

    mfrc.setSPIConfig();
    mfrc.PCD_Init();

    // set key

    for (byte i = 0; i < 6; i++)
    {
      key.keyByte[i] = 0xFF;
    }
    success = false;
    
}

bool
MFRC522_Delegate::readable_card_detected()
{
    return mfrc.PICC_IsNewCardPresent() && mfrc.PICC_ReadCardSerial();
}

void
MFRC522_Delegate::write_to_rc(const string &text, const byte &start_block)
{
    vector<byte> text_byte;

    dump_string_to_byte_array(text, text_byte);

    byte tmp[64];

   for (size_t i=0; i < text_byte.size(); ++i)
   {
       tmp[i] = text_byte[i];
   }

   status = static_cast<MFRC522::StatusCode>(mfrc.MIFARE_Write(start_block, tmp, 32));
   if (status != MFRC522::STATUS_OK)
   {
       cout << "MIFARE_Write() failed: " << mfrc.GetStatusCodeName(status) << endl;
   }
}

void
MFRC522_Delegate::uid_bytes_to_string(MFRC522 &mfrc522)
{
    stringstream stream;

    for (byte i=0; i < mfrc522.uid.size; ++i)
    {
        stream << static_cast<int>(mfrc522.uid.uidByte[i]);
        /*
        if (mfrc522.uid.uidByte[i] < 0x10)
        {
            stream << " 0" << hex << static_cast<int>(mfrc522.uid.uidByte[i]);
        }
        else
        {
            stream << "" << hex << static_cast<int>(mfrc522.uid.uidByte[i]);
        }
        */
    }

    card_uid = stream.str();
    // log_mfrc_status(mfrc522);
}

void
MFRC522_Delegate::log_mfrc_status(MFRC522 &mfrc522)
{
    byte trailerBlock   = 11;

    cout << "UID: " << card_uid << endl;
    cout << "PICC type: " << mfrc.PICC_GetTypeName(mfrc522.PICC_GetType(mfrc522.uid.sak)) << endl;
    cout << "SAK: " << static_cast<int>(mfrc522.uid.sak) << endl;

    status = static_cast<MFRC522::StatusCode>(mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
                                                                    trailerBlock, &key, &(mfrc522.uid)
                                                                    ));

    cout << "Status: " << mfrc522.GetStatusCodeName(status) << endl;
}

void
MFRC522_Delegate::dump_string_to_byte_array(const string text, vector<byte> &array)
{
    for (size_t i=0; i < text.size(); ++i)
    {
        stringstream stream;
        stream << "0x" << hex << static_cast<int>(text[i]);
        string hex = stream.str();
        array.push_back(static_cast<byte>(text[i]));
    }

    for (size_t i=0; i < (16-text.size()); ++i)
    {
        array.push_back(0x00);
    }
}


void
MFRC522_Delegate::halt_picc()
{
    mfrc.PICC_HaltA();
    mfrc.PCD_StopCrypto1();
}

MFRC522
MFRC522_Delegate::get_mfrc()
{
    return mfrc;
}

string
MFRC522_Delegate::get_card_uid()
{
    return card_uid;
}
