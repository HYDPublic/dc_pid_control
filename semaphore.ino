bool semaphore = true;

void semGive()
{
  semaphore = true;
  Serial.println("semGive");
}

unsigned char semTake(unsigned int timeout_ms)
{
  Serial.print("semTake ");Serial.print(semaphore);

  if (semaphore == true) {
    semaphore = false;
    return 0;
  }

  return 1;
}

