/*
 ----------------------------------------------------------
|   PROJETO  : TACOMETRO 8 INPUT via rede 485               |
|   AUTOR    : STENIO RODRIGUES                             |
|   DATA     : 02/05/2022                                   |
|   OBJETIVO :                                              |
|             Apontar conversão de sensor tipo encoder      |
|               escala 100p/volta para RPM via rede 485     |
|              aplicação industrial geral.                  |
|   Atualização: Método de rede, sequenciar  respostas      |
|                em cascata dos clientes baseado na         |
|               quantidade de clientes e Mudança de I/O     |
|                                                           |
 ----------------------------------------------------------
   Organização do código.
    // =================================================================================
    // --- Bibliotecas Auxiliares       --- //
    // =================================================================================
    // --- Mapeamento de Hardware       --- //
    // =================================================================================
    // --- Variáveis Globais            --- //
    // =================================================================================
    // --- Instâncias                   --- //
    // =================================================================================
    // --- Protótipo das Funções        --- //
    // =================================================================================
    // --- Interrupções                 --- //
    // =================================================================================
    // --- Configurações Iniciais SETUP --- //
    // =================================================================================
    // --- Loop Infinito                --- //
    // =================================================================================
    // --- Desenvolvimento das Funções  --- //
    // =================================================================================
 */

// =================================================================================
    // --- Mapeamento de Hardware       --- //
const int PinB0   = A3;
const int PinB1   = A4;
const int PinB2   = A5;
const int ledPin  = 13;      // the number of the LED pin
const int bit_end_0  = 10; 
const int bit_end_1  = 11;
const int bit_end_2  = 12;  
// =================================================================================
    // --- Variáveis Globais            --- //
 int teste = 0;
 

// =================================================================================
    // --- Protótipo das Funções        --- //
// Função que alterar porta digitais para definir qual cliente responde solicitação
void Chama_cliente  (int end_bit_client );
// Retorna qual endereço de cliente está falando
int Qual_cliente    (                   );




// =================================================================================
    // --- Configurações Iniciais SETUP --- //
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode     (ledPin   , OUTPUT         );
  pinMode     (bit_end_0, OUTPUT         );
  pinMode     (bit_end_1, OUTPUT         );
  pinMode     (bit_end_2, OUTPUT         );

  pinMode     (PinB0    , INPUT          );
  pinMode     (PinB1    , INPUT          );
  pinMode     (PinB2    , INPUT          );

  digitalWrite(bit_end_0, LOW           );
  digitalWrite(bit_end_1, LOW           );
  digitalWrite(bit_end_2, LOW           );
  Serial.begin   (9600);
  Serial.println ("Arquivo:PROJETO-TACO-teste-imput-rede...\n");
  
}// end SETUP

// the loop function runs over and over again forever
void loop() {
/*
 for (size_t i = 1; i <= 4; i++)
 {
   delay(1000);
   Chama_cliente (i);
   Serial.println (i);
   delay(2000);
 }// end for
*/

      teste = Qual_cliente();
      Serial.print    ("CLIENTE: "    );
      Serial.println  (teste          );
      delay           (500            ); 

      Serial.println    ("B2  B1  B0:" );
        Serial.print  (digitalRead(PinB2));
      Serial.print      (" :  " );  
        Serial.print  (digitalRead(PinB1)); 
        Serial.print      (" :  " );  
        Serial.print  (digitalRead(PinB0)); 
        Serial.println      (" " );  


/*
 O cliente 1  envia via rede sua leitura
 e muda o estado das portas para numero 2 espera o timeout
 dempois muda estado das portas para t3... até voltar ele de novo.
*/

}// END LOOP

// =================================================================================
    // --- Desenvolvimento das Funções  --- //
void Chama_cliente (int end_bit_client){
  /* TABELA VERDADE ENDEREÇO CLIENTE
  bit_end_2 bit_end_1 bit_end_0 
      0         0         1       /// cliente 1
      0         1         0       /// cliente 2
      0         1         1       /// cliente 3
      1         0         0       /// cliente 4 */
     switch (end_bit_client)
                    { 
                        case  1:
                                digitalWrite(bit_end_0, LOW    ); 
                                digitalWrite(bit_end_1, HIGH   );
                                digitalWrite(bit_end_2, HIGH   );
                                break;
                        case  2:
                                digitalWrite(bit_end_0, HIGH   );
                                digitalWrite(bit_end_1, LOW    ); 
                                digitalWrite(bit_end_2, HIGH   );
                                break;
                        case  3:
                                digitalWrite(bit_end_0, LOW    ); 
                                digitalWrite(bit_end_1, LOW    ); 
                                digitalWrite(bit_end_2, HIGH   );
                                break;
                        case  4:
                                digitalWrite(bit_end_0, HIGH   );
                                digitalWrite(bit_end_1, HIGH   );
                                digitalWrite(bit_end_2, LOW    ); 
                                break;
                        default:
                                digitalWrite(bit_end_0, HIGH   );
                                digitalWrite(bit_end_1, HIGH   );
                                digitalWrite(bit_end_2, HIGH   );
                                break;
                    } // end switch
} // End Chama_cliente_2
// =================================================================================

int Qual_cliente(){
  /* TABELA VERDADE ENDEREÇO CLIENTE
  bit_end_2 bit_end_1 bit_end_0 
      0         0         1       /// cliente 1
      0         1         0       /// cliente 2
      0         1         1       /// cliente 3
      1         0         0       /// cliente 4 */
  int qual_cliente = 0;
  int leitura_bit_0 = digitalRead(PinB0);
  int leitura_bit_1 = digitalRead(PinB1);
  int leitura_bit_2 = digitalRead(PinB2);
  if (leitura_bit_2==0 && leitura_bit_1==0 && leitura_bit_0==1) qual_cliente = 1;
  if (leitura_bit_2==0 && leitura_bit_1==1 && leitura_bit_0==0) qual_cliente = 2;
  if (leitura_bit_2==0 && leitura_bit_1==1 && leitura_bit_0==1) qual_cliente = 3;
  if (leitura_bit_2==1 && leitura_bit_1==0 && leitura_bit_0==0) qual_cliente = 4;
  if (leitura_bit_2==1 && leitura_bit_1==1 && leitura_bit_0==1) qual_cliente = 9;

  return  qual_cliente;

}// end Qual_cliente