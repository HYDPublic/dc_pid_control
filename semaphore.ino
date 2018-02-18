bool semaphore = true;

void semGive()
{
  semaphore = true;
  myRS485Serial.println("semGive");
}

unsigned char semTake(unsigned int timeout_ms)
{
  myRS485Serial.print("semTake ");myRS485Serial.print(semaphore);

  if (semaphore == true) {
    semaphore = false;
    return 0;
  }

  return 1;
}

