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
// --- Bibliotecas Auxiliares       --- //
#include <avr/io.h>  // wachdog
#include <avr/wdt.h> // wachdog

// =================================================================================
// --- Mapeamento de Hardware       --- //
//Portas analogicas     A0 A1 A2 A3 A4 A5
//Portas Digitais 12 13 14 15 16 17 18 19
const int PinB0 = A3;
const int PinB1 = A4;
const int PinB2 = A5;

const int bit_end_0 = 10;
const int bit_end_1 = 11;
const int bit_end_2 = 12;
const int MASTER = 13;

#define Pbit_0 14 // A0 
#define Pbit_1 15 // A1
#define Pbit_2 16 // A2 

// ---  mapeamento de hardware      --- //
const byte pin_02 = (1 << 2); //2;
const byte pin_03 = (1 << 3);
const byte pin_04 = (1 << 4);
const byte pin_05 = (1 << 5);
const byte pin_06 = (1 << 6);
const byte pin_07 = (1 << 7);
const byte pin_08 = (1 << 0);
const byte pin_09 = (1 << 1);
// =================================================================================
// --- Variáveis Globais            --- //


unsigned long timeold           = 0;        //volatile byte pulsos;
unsigned int  pulsos_por_volta  = 200;

int 
              teste             = 0,
              end_cliente_num   = 0,
              conversor         = 1,
              bit_0             = 0,
              bit_1             = 0,
              bit_2             = 0,
              pulsos_2          = 0,
              pulsos_3          = 0,
              pulsos_4          = 0,
              pulsos_5          = 0,
              pulsos_6          = 0,
              pulsos_7          = 0,
              pulsos_8          = 0,
              pulsos_9          = 0;

String
              str         = "",                                   // string de buffer de caracteres rede 485
              str_sensor  = "",
              str_BASE    = "",
              end_CONF    = "";

volatile byte state_D2          = LOW, 
              state_D3          = LOW,
              state_D4          = LOW,
              state_D5          = LOW,
              state_D6          = LOW,
              state_D7          = LOW,
              state_D8          = LOW,
              state_D9          = LOW;

// =================================================================================
// --- Protótipo das Funções        --- //
// Função que alterar porta digitais para definir qual cliente responde solicitação
void Chama_cliente          (int end_bit_client                                         );
// Retorna qual endereço de cliente está falando
int  Qual_cliente           (                                                           );
// Retorna qual endereço de cliente está falando para teste
void Debug_qual_cliente     (                                                           );
// Gerador de endereços de clientes na rede para teste
void Debug_gerador_endereco (                                                           );
// Define endereço do micro baseado nos pinos de entrada baseado em binario 
void Gerenciador_endereco   (                                                           );
// Contador de pulsos
int  Contador               (                                                           );
// Monta o vetor com as leituras de pulsos convertidas em RPM
void Vetor_RPM              (float ref_temp,int n_pulso,int n_pulso_volta,int end_sensor);
// Abre a rede 485 e envia dados.
void Proto_485_V2           (String end_cliente, String msg_cliente                     );
// Debug serial com temperatura convertida
void Imprimir_serial_temp   (                                                           );

// =================================================================================
// --- Configurações Iniciais SETUP --- //
void setup() {
   wdt_disable  (                     ); // desativa o dog
   wdt_enable   (WDTO_2S              ); //    /*#define WDTO_15MS,30MS,60MS,120MS,250MS,500MS,1S,2S,4S,8S*/
   // pino responsável por habilitar comunicação serial 485
   pinMode      (MASTER, OUTPUT       );
   // saida endereço de rede
   pinMode      (bit_end_0, OUTPUT    );
   pinMode      (bit_end_1, OUTPUT    );
   pinMode      (bit_end_2, OUTPUT    );
   // entradas endereço da rede
   pinMode      (PinB0, INPUT_PULLUP  );
   pinMode      (PinB1, INPUT_PULLUP  );
   pinMode      (PinB2, INPUT_PULLUP  );
   // configuração entrada sensores

   DDRD &= ~pin_02; // pin_02 o bit 2 do registrador ENTRADA DIGITAL PIN  REGISTRADORES  
   DDRD &= ~pin_03;
   DDRD &= ~pin_04;
   DDRD &= ~pin_05;
   DDRD &= ~pin_06;
   DDRD &= ~pin_07;
   DDRB &= ~pin_08; // note que mudou o registrador de ddrD para ddrB consulte pinout do atmega328PU
   DDRB &= ~pin_09; // pin_09 o bit 9 do registrador ENTRADA DIGITAL PIN  REGISTRADORES 

   PORTD |= pin_02; // ATIVANDO PULLUP CORRIGIR BUG LEITURA FALSA
   PORTD |= pin_03;
   PORTD |= pin_04;
   PORTD |= pin_05;
   PORTD |= pin_06;
   PORTD |= pin_07;
   PORTB |= pin_08; // note mudança do portD para portB
   PORTB |= pin_09;
   // entrada endereço interno cliente
   pinMode(Pbit_0, INPUT_PULLUP);
   pinMode(Pbit_1, INPUT_PULLUP);
   pinMode(Pbit_2, INPUT_PULLUP);
   // armazenando endereço configurado para essa unidade
   bit_0 = digitalRead(Pbit_0);
   bit_1 = digitalRead(Pbit_1);
   bit_2 = digitalRead(Pbit_2);

   // partindo saidas digitais nivel baixo
   digitalWrite(bit_end_0, LOW);
   digitalWrite(bit_end_1, LOW);
   digitalWrite(bit_end_2, LOW);
   digitalWrite(MASTER, LOW);
   Serial.begin(9600);
   Serial.println("Arquivo:Teste_rede_opto...\n");
   Gerenciador_endereco();
   wdt_reset(); // reinicia contador de cachorrão!!!!
} // end SETUP

// the loop function runs over and over again forever
   /* O cliente 1  envia via rede sua leitura
      e muda o estado das portas para numero 2 espera o timeout
      dempois muda estado das portas para t3... até voltar ele de novo. */

void loop() {
  wdt_reset               (     );
///////////////////////////////////////////////////////////////////
   /////// ---  Debug gerador de cliente --- ///////
   // Debug_gerador_endereco ();
   /////// ---  Debug identificador de cliente --- ///////
   teste = Qual_cliente  (     );
   Contador              (     );// SEMPRE ATIVO
   if (end_cliente_num == teste )
      {
        //Debug_qual_cliente   (     );
          Imprimir_serial_temp (     );
      }//end if
///////////////////////////////////////////////////////////////////

} // END LOOP

// =================================================================================
// --- Desenvolvimento das Funções  --- //
void Gerenciador_endereco   () {
   // clinte com todos sensores definir endereço
   if (bit_2 == 0 && bit_1 == 0 && bit_0 == 1) end_cliente_num = 1;
   if (bit_2 == 0 && bit_1 == 1 && bit_0 == 0) end_cliente_num = 2;
   if (bit_2 == 0 && bit_1 == 1 && bit_0 == 1) end_cliente_num = 3;
   if (bit_2 == 1 && bit_1 == 0 && bit_0 == 0) end_cliente_num = 4;
   if (bit_2 == 1 && bit_1 == 0 && bit_0 == 1) end_cliente_num = 5;
   if (bit_2 == 1 && bit_1 == 1 && bit_0 == 0) end_cliente_num = 6;
   if (bit_2 == 1 && bit_1 == 1 && bit_0 == 1) end_cliente_num = 7;
   /*
   if (bit_3 == 1 && bit_2 == 0 && bit_1 == 0 && bit_0 == 0 ) end_cliente_num = 8;
   if (bit_3 == 1 && bit_2 == 0 && bit_1 == 0 && bit_0 == 1 ) end_cliente_num = 9;
   if (bit_3 == 1 && bit_2 == 0 && bit_1 == 1 && bit_0 == 0 ) end_cliente_num = 10;
   if (bit_3 == 1 && bit_2 == 0 && bit_1 == 1 && bit_0 == 1 ) end_cliente_num = 11;
   if (bit_3 == 1 && bit_2 == 1 && bit_1 == 0 && bit_0 == 0 ) end_cliente_num = 12;
   if (bit_3 == 1 && bit_2 == 1 && bit_1 == 0 && bit_0 == 1 ) end_cliente_num = 13;
   if (bit_3 == 1 && bit_2 == 1 && bit_1 == 1 && bit_0 == 0 ) end_cliente_num = 14;
   if (bit_3 == 1 && bit_2 == 1 && bit_1 == 1 && bit_0 == 1 ) end_cliente_num = 15;*/
      Serial.print("ENDERECO CONVERSOR: ");
      Serial.println(end_cliente_num);
      switch (end_cliente_num) {
              case 1:
                  end_CONF = "P001*";
                  break;
              case 2:
                  end_CONF = "P002*";
                  break;
              case 3:
                  end_CONF = "P003*";
                  break;
              case 4:
                  end_CONF = "P004*";
                  break;
              case 5:
                  end_CONF = "P005*";
                  break;
              case 6:
                  end_CONF = "P006*";
                  break;
              case 7:
                  end_CONF = "P007*";
                  break;
              case 8:
                  end_CONF = "P008*";
                  break;
              case 9:
                  end_CONF = "P009*";
                  break;
              case 10:
                  end_CONF = "P010*";
                  break;
              case 11:
                  end_CONF = "P011*";
                  break;
              case 12:
                  end_CONF = "P012*";
                  break;
              case 13:
                  end_CONF = "P013*";
                  break;
              case 14:
                  end_CONF = "P014*";
                  break;
              case 15:
                  end_CONF = "P015*";
                  break;
              default:
                  Serial.print("ERRO AO CONFIGURAR ENDERECO CONVERSOR: ");
                  wdt_reset();
                  break;
      } // END SWITCH
} // end Gerenciador_endereco
// =================================================================================
void Debug_gerador_endereco () {
   for (size_t i = 1; i <= 4; i++) {
      delay(1000);
      Chama_cliente(i);
      Serial.println(i);
      delay(2000);
   } // end for
} // End gerador de endereços sequenciais
// =================================================================================
void Debug_qual_cliente     () {
   //teste = Qual_cliente();
   Serial.print("CLIENTE: ");
   Serial.println(teste);
   delay(500);
   /*
      Serial.println("B2  B1  B0:");
      Serial.print(digitalRead(PinB2));
      Serial.print(" :  ");
      Serial.print(digitalRead(PinB1));
      Serial.print(" :  ");
      Serial.print(digitalRead(PinB0));
      Serial.println(" "); */
} // fim debug entrada endereço rede
// =================================================================================
void Chama_cliente          (int end_bit_client) {
   /* TABELA VERDADE ENDEREÇO CLIENTE
   bit_end_2 bit_end_1 bit_end_0 
       0         0         1       /// cliente 1
       0         1         0       /// cliente 2
       0         1         1       /// cliente 3
       1         0         0       /// cliente 4 */
   switch (end_bit_client) {
            case 1:
                digitalWrite(bit_end_0, LOW);
                digitalWrite(bit_end_1, HIGH);
                digitalWrite(bit_end_2, HIGH);
                break;
            case 2:
                digitalWrite(bit_end_0, HIGH);
                digitalWrite(bit_end_1, LOW);
                digitalWrite(bit_end_2, HIGH);
                break;
            case 3:
                digitalWrite(bit_end_0, LOW);
                digitalWrite(bit_end_1, LOW);
                digitalWrite(bit_end_2, HIGH);
                break;
            case 4:
                digitalWrite(bit_end_0, HIGH);
                digitalWrite(bit_end_1, HIGH);
                digitalWrite(bit_end_2, LOW);
                break;
            default:
                digitalWrite(bit_end_0, HIGH);
                digitalWrite(bit_end_1, HIGH);
                digitalWrite(bit_end_2, HIGH);
                break;
   } // end switch
} // End Chama_cliente_2
// =================================================================================
int Qual_cliente            () {
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
      if (leitura_bit_2 == 0 && leitura_bit_1 == 0 && leitura_bit_0 == 1) qual_cliente = 1;
      if (leitura_bit_2 == 0 && leitura_bit_1 == 1 && leitura_bit_0 == 0) qual_cliente = 2;
      if (leitura_bit_2 == 0 && leitura_bit_1 == 1 && leitura_bit_0 == 1) qual_cliente = 3;
      if (leitura_bit_2 == 1 && leitura_bit_1 == 0 && leitura_bit_0 == 0) qual_cliente = 4;
      if (leitura_bit_2 == 1 && leitura_bit_1 == 1 && leitura_bit_0 == 1) qual_cliente = 9;

   return qual_cliente;
} // end Qual_cliente
// =================================================================================
int Contador                (){
   if (  state_D2 != (PIND & pin_02) ) {
         state_D2  = (PIND & pin_02); // atualizando a leitura atual do sensor
         pulsos_2++;
   } // end if
   if (  state_D3 != (PIND & pin_03) ){
         state_D3  = (PIND & pin_03);
         pulsos_3++;
   }// end if
   if (  state_D4 != (PIND & pin_04) ){
         state_D4  = (PIND & pin_04);
         pulsos_4++;
   }// end if
   if (  state_D5 != (PIND & pin_05) ){
         state_D5  = (PIND & pin_05);
         pulsos_5++;
   }// end if
   if (  state_D6 != (PIND & pin_06) ){
         state_D6  = (PIND & pin_06);
         pulsos_6++;
   }// end if
   if (  state_D7 != (PIND & pin_07) ){
         state_D7  = (PIND & pin_07);
         pulsos_7++;
   }// end if
   if (  state_D8 != (PINB & pin_08) ){
         state_D8  = (PINB & pin_08);
         pulsos_8++;
   }// end if
   if (  state_D9 != (PINB & pin_09) ){
         state_D9  = (PINB & pin_09);
         pulsos_9++;
   }// end if
}// end  contador()
// =================================================================================
void Vetor_RPM              (float ref_temp, int n_pulso, int n_pulso_volta, int end_sensor){
         float referencia  = ref_temp;
         int pulsos        = n_pulso;
         int pulsos_volta  = n_pulso_volta;
         float rpm         = 0.0;
         String str_DADOS="",str_END="";
         rpm = ( (float)(referencia) * (float)(pulsos) )/( (float)(pulsos_volta) );
         //str_DADOS = String(rpm);
            if (rpm<0) rpm = 0;
            ////if (rpm>98) rpm = 98;
            if (rpm <10)str_DADOS = String(rpm, 2);
            if (rpm>=10)str_DADOS = String(rpm, 1);
            switch (end_sensor){
               case 1:
                  str_END = "A";
                  break;
               case 2:
                  str_END = "B";
                  break;
               case 3:
                  str_END = "C";
                  break;
               case 4:
                  str_END = "D";
                  break;
               case 5:
                  str_END = "E";
                  break;
               case 6:
                  str_END = "F";
                  break;
               case 7:
                  str_END = "G";
                  break;
               case 8:
                  str_END = "H";
                  break;
               default:
                  break;
            } // END SWITCH
         str_BASE.concat(str_END);
         str_BASE.concat(str_DADOS);
}// end vetor_RPM
// =================================================================================
void Proto_485_V2           (String end_cliente, String msg_cliente             ){
    digitalWrite  (MASTER, HIGH  );
    delay         (50            );
    Serial.print  (msg_cliente   );
    Serial.flush  (              );
    Serial.print  (end_cliente   );
    Serial.flush  (              );
    delay         (50            );
    digitalWrite  (MASTER, LOW   );
    Serial.println(""            );
} // end Proto_485_V2
// =================================================================================
void Imprimir_serial_temp   (                                                   ){
   String  str_DADOS="",str_END="";
      if (millis() - timeold >= 1000){
         int long timeold_2 = (millis() - timeold);
         //rpm = 60*1000 / t * actualREV => rpm = 60*1000 / (millis() - time ) * REV/2 OR rpm = 30*1000 / (millis() - time) * REV; //n = f . 60 / p
         float referencia =  60000 / timeold_2; //converte tempo de minutos para microsegundos
         int end_sensor=1;
         int pulso=0; 
         conversor = 1; 
            for (size_t i = 1; i <= 8; i++){ 
               end_sensor = i;
               switch (i)
                      {
                          case 1:
                            pulso = pulsos_2;
                            Vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                            break;
                          case 2:
                            pulso = pulsos_3;
                            Vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                            break;
                          case 3:
                            pulso = pulsos_4;
                            Vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                            break;
                          case 4:
                            pulso = pulsos_5;
                            Vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                            break;
                          case 5:
                            pulso = pulsos_6;
                            Vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                            break;
                          case 6:
                            pulso = pulsos_7;
                            Vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                            break;
                          case 7:
                            pulso = pulsos_7;
                            Vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                            break;
                          case 8:
                            pulso = pulsos_8;
                            Vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                            break;
                          case 9:
                            pulso = pulsos_9;
                            Vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                            break;
                          default:
                            break;
                      }// end witch          
            }// end for
         //Serial.print  ("RPM_"         );
         //Serial.print  (end_CONF      );
         //Serial.println  (str_BASE       );
            String msg = str_BASE;
            Proto_485_V2(end_CONF, msg);
         //Serial.print  (" Delta_T: "   );
         //Serial.print  (timeold_2/1000 );
         //Serial.println("s"            );
         pulsos_2=0,pulsos_3=0,pulsos_4=0,pulsos_5=0,pulsos_6=0,pulsos_7=0,pulsos_8=0,pulsos_9=0, timeold_2 = 0;
         str_BASE= "";
         timeold = millis();
         Contador();
      }// end if  
}// end  imprimir_serial_temp
// =================================================================================
