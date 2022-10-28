TaskHandle_t Task1;
TaskHandle_t Task2;

int currentTime;

void setup() {
  currentTime = millis();
  Serial.begin(115200);
xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 
  

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 


}


void Task1code( void * parameter) {
  
  for(;;) {
    Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  delay(500);

  }
  vTaskDelete(NULL);
}

void Task2code( void * pvParameters ){
  for(;;) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  delay(1500);
  }
  vTaskDelete(NULL);
}
void loop() {
//  if (millis() - currentTime > 5000) {
//    vTaskDelete(Task1);
//    vTaskDelete(Task2);
//  }
  // put your main code here, to run repeatedly:
  

}
