/*
 ----------------------------------------------------------
|   PROJETO  : TACOMETRO 8 INPUT via rede 485              |
|   AUTOR    : STENIO RODRIGUES                            |
|   DATA     : 01/02/2022                                  |
|   OBJETIVO :                                             |
|             Apontar conversão de sensor tipo encoder     |
|               escala 100p/volta para RPM via rede 485    |
|              aplicação industrial geral.                 |
 ----------------------------------------------------------
      Organizando código.
    // =================================================================================
    // --- Bibliotecas Auxiliares       --- //
    // =================================================================================
    // --- Mapeamento de Hardware       --- //
    // =================================================================================
    // --- Instâncias                   --- //
    // =================================================================================
    // --- Protótipo das Funções        --- //
    // =================================================================================
    // --- Variáveis Globais            --- //
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

// --- Bibliotecas Auxiliares       --- //
#include <avr/io.h>  // wachdog
#include <avr/wdt.h> // wachdog

// =================================================================================
// ---  mapeamento de hardware      --- //
const byte pin_02 = (1 << 2); //2;
const byte pin_03 = (1 << 3);
const byte pin_04 = (1 << 4);
const byte pin_05 = (1 << 5);
const byte pin_06 = (1 << 6);
const byte pin_07 = (1 << 7);
const byte pin_08 = (1 << 0);
const byte pin_09 = (1 << 1);
//Portas analogicas     A0 A1 A2 A3 A4 A5
//Portas Digitais 12 13 14 15 16 17 18 19
int MASTER = 13;
#define Pbit_0 14    // A0 
#define Pbit_1 15    // A1
#define Pbit_2 16    // A2 
#define Pbit_3 17    // A3 
#define Pbit_A 18    // A4 
#define Pbit_B 19    // A5 
// =================================================================================
// --- Variáveis Globais            --- //
int   pulsos_2          = 0,
      ciclos1           = 0,
      pulsos_3          = 0,
      pulsos_4          = 0,
      pulsos_5          = 0,
      conta_caracter    = 0,
      pulsos_6          = 0,
      pulsos_7          = 0,
      pulsos_8          = 0,
      pulsos_9          = 0,
      temp              = 0,
      ident_sensor_G    = 0;

//volatile byte pulsos;
unsigned long timeold   = 0;
//Altere o numero abaixo de acordo com o seu disco encoder
unsigned int   pulsos_por_volta  = 200;
volatile byte  state_D2          = LOW, 
               state_D3          = LOW,
               state_D4          = LOW,
               state_D5          = LOW,
               state_D6          = LOW,
               state_D7          = LOW,
               state_D8          = LOW,
               state_D9          = LOW;

#define num 10 //número de iterações da média móvel
float           results_G             = 0.00,               // armazena leitura conversor A/D
                results1_G            = 0.00,
                results2_G            = 0.00,
                results3_G            = 0.00,
                results4_G            = 0.00,
                results5_G            = 0.00,
                results6_G            = 0.00,
                results7_G            = 0.00,
                results8_G            = 0.00,
                values_G[num],                              // vetor de iteração de média móvel
                values1_G[num],
                values2_G[num],
                values3_G[num],
                values4_G[num],
                values5_G[num],
                values6_G[num],
                values7_G[num],
                values8_G[num];

String   str         = "",                                   // string de buffer de caracteres rede 485
         str_sensor  = "",
         str_BASE    = "",
         end_CONF    = "";

const int tam_msg = 62; 
char    charRecebida[tam_msg],                              // vetor de buffer rede 485
        str1[tam_msg],
        char_pulsos[9];
int   end_cliente_num   =0, 
      conversor         =1,
      bit_0,
      bit_1,
      bit_2,
      bit_3,
      bit_A,
      bit_B;
// achar portas para fazer o endereçamento

// =================================================================================
// --- Instâncias                   --- //

// --- Protótipo das Funções        --- //

// Pra que server
// tantos códigos?
// Se a vida
// não é programada
// e as melhores coisas
// não tem lógica

// Contador de pulsos
int contador                     (                                               );
// Converte pulsos em RPM virou DEBUG
void tacometro                   (String end_cliente                             );
// Simula pulsos para teste sem usar o motor
void simu_Sensor                 (int temp                                       );
// Realiza a média móvel de forma genérica
float moving_average             (float sig_G, int identidade                    );
// Chama média móvel da leitura do conversor de acordo com quantidade de sensores
void media_leitura               (int conversor                                  );
//Coloca marcador de endereço no vetor de dados 
void Proto_485_V3                (int ident_sensor_G                             );
// Faz tratamento básico caso algum sensor não responda preenche vetor com zero.
void Proto_485_V4                (char *testar, int tamanho                      );
// Abre a rede 485 e envia dados.
void Proto_485_V2                (String end_cliente, String msg_cliente         );
// define endereço do micro baseado nos pinos de entrada baseado em binario e com marcador de quantos sensores online
void Gerenciador_endereco        (                                               );
// envia solicitação na rede 485 para cliente responder dados, analisa ultimo cliente que enviou
void Gerenciador_rede            (                                               );
//debug serial com temperatura convertida
extern void imprimir_serial_temp (                                               );
//Monta o vetor com as leituras de pulsos convertidas em RPM
void vetor_RPM                   (float ref_temp, int n_pulso, int n_pulso_volta, int end_sensor);
// Recebe solicitação via rede 485
void Proto_485_V5                ();
// =================================================================================
// --- Configurações Iniciais SETUP --- //
void setup() {
   wdt_disable(); // desativa o dog
   wdt_enable(WDTO_2S); //    /*#define WDTO_15MS,30MS,60MS,120MS,250MS,500MS,1S,2S,4S,8S*/

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

   pinMode     (Pbit_0, INPUT_PULLUP   );
   pinMode     (Pbit_1, INPUT_PULLUP   );
   pinMode     (Pbit_2, INPUT_PULLUP   );
   pinMode     (Pbit_3, INPUT_PULLUP   );
   pinMode     (Pbit_A, INPUT_PULLUP   );
   pinMode     (Pbit_B, INPUT_PULLUP   );

    bit_0  = digitalRead(Pbit_0); // logica invertida para dar certo com montagem!
    bit_1  = digitalRead(Pbit_1); 
    bit_2  = digitalRead(Pbit_2); 
    bit_3  = digitalRead(Pbit_3);
    bit_A  = digitalRead(Pbit_A); 
    bit_B  = digitalRead(Pbit_B);

   pinMode     (MASTER, OUTPUT         ); // usado na função de simulada
   digitalWrite(MASTER, LOW            );

   temp = 8000; //the range 3 microseconds 
   Serial.begin   (9600);
   Serial.println ("Arquivo:PROJETO-TACOMETRO-8-INPUT-485-V1...\n");
   //Serial.println ("funcao: Proto_485_V5  (  );\n");
   //Serial.println("funcao: simu_Sensor  (temp );\n");
   //Serial.println("funcao: contador e imprimir_serial_temp;\n");

   Gerenciador_endereco();
   wdt_reset();
   /* -------- MELHORIAS ------- //
      criar condição quando o sensor falhar e nao enviar nada na conversao */

} // end setup
// =================================================================================
// --- Loop Infinito                --- //
void loop() {
   wdt_reset             (     );
   if (ciclos1 > 1000) { // reinicia a cada 1000 SOLICITAÇÕES !
            ciclos1 = 0;
            wdt_disable     (               ); // desativa o dog
            wdt_enable      (WDTO_250MS     ); // ativa o wachdog
            Serial.println  ("R"            );
            delay           (1000           );
   }// end if

   contador              (     );// SEMPRE ATIVO
   imprimir_serial_temp  (     );
     //Proto_485_V5          (     ); // RECEBE SOLICITAÇÃO REDE 485
       
//Gerenciador_rede      (     ); // LIMPA VETOR E ERROS DE DEMANDA
// INICIO DESENVOLVIMENTO DE PROTOCOLO DE REDE SOBRE DEMANDA
// FIM DESENVOLVIMENTO PROTROCOLO DE REDE 
/////// INICIO    bloco simulador de sensor ///////
   //temp=20000;
   //simu_Sensor  (temp );
/////// FIM       bloco simulador de sensor ///////

/*  
DIA  12/04  ////////////   FALTA RESOLVER BUG DE RESPONDER A MENSAGEM DE SOLICITAÇÃO REENVIANDO O QUE FOI RECEBIDO NO CANAL
 
SEQUENCIA DO PROCESSO REDE
   OK RESPONDER A SOLICITAÇÃO VIA REDE

SEQUENCIA DO PROCESSSO VETOR MEDIDO
   OK RESPONDER SOLICITAÇÃO INTERNA

//////////////////////////////////////////////////////
DIA 07/04   ////////////
SEQUENCIA INICIAL PROCESSO
   OK INICIAR CONTADOR DE WATCHDOG 5 MINUTOS

SEQUENCIA DO PROCESSO REDE
   OK RECEBER A SOLICITAÇÃO VIA REDE 

SEQUENCIA DO PROCESSSO VETOR MEDIDO
//////////////////////////////////////////////////////
DIA 06/04
SEQUENCIA INICIAL PROCESSO
   OK IDENTIFICAR ENDEREÇO CONFIGURADO 
   OK RESPONDIDO NA VARIAVEL end_cliente_num COM NUMEROS
   OK RESPONDIDO NA VARIAVEL end_CONF        COM STRING
   OK IMPRIMIR NA SERIAL LOCAL QUAL A VERSAO DO SOFTWARE E QUAIS FUNÇOES ATIVAS
   
SEQUENCIA DO PROCESSO REDE
   OK RECEBER A SOLICITAÇÃO
   OK DEFINIR PARA QUAL CLIENTE É A MENSAGEM
   OK COMPARAR COM O ENDEREÇO CONFIGURADO
   OK LIMPAR VETOR DA MESAGEM RESPONDIDA E O BUFFER DA REDE

SEQUENCIA DO PROCESSSO VETOR MEDIDO
   OK MONTAR VETOR PADRAO PREENCHIDO COM TAMANHO E MENSAGEM INICIAL
   OK COLETAR LEITURAS DAS ENTRADAS E DIVIDIR PELO INTERVALO DE TEMPO RESULTADO EM RPM 
   OK PREPARAR VETOR DE RESPOSTA COM ENDEREÇO CONFIGURADO E LEITURAS ATUALIZADAS          */
 
} // end loop
// =================================================================================
// --- Desenvolvimento das Funções  --- //

// =================================================================================
void Proto_485_V2           (String end_cliente, String msg_cliente             ){
    digitalWrite  (MASTER, HIGH  );
    delay         (50             );
    Serial.print  (msg_cliente   );
    Serial.flush  (              );
    Serial.print  (end_cliente   );
    Serial.flush  (              );
    delay         (50            );
    digitalWrite(MASTER, LOW     );
} // end Proto_485_V2
// =================================================================================

void Proto_485_V3           (int ident_sensor_G                                 ){
    int P_sensor_G  = ident_sensor_G;
    switch (ident_sensor_G) 
      {
            case 1:
                str_sensor = "A";
                break;
            case 2:
                str_sensor = "B"; 
                break;
            case 3:
                str_sensor = "C"; 
                break;
            case 4:
                str_sensor = "D"; 
                break;
            case 5:
                str_sensor = "E"; 
                break;
            case 6:
                str_sensor = "F"; 
                break;
            case 7:
                str_sensor = "G"; 
                break;
            case 8:
                str_sensor = "H"; 
                break;
      } // end SWITCH
} // end Proto_485_V3
// =================================================================================

void Proto_485_V4           (char *testar, int tamanho                          ){
      int tam = tamanho;
      char manobra [tam];
      for (size_t i = 0; i <= 41; i++) manobra [i] = testar [i];
        //         1   2   3   4   5   6   7   8
        //String  "A000B000C000D000E000F000G000H000";
            if (str1[0 ] != 'A' ){
                manobra[0 ] = 'A';
                manobra[1 ] = '0';
                manobra[2 ] = '.';
                manobra[3 ] = '0';
                manobra[4 ] = '0';
            } // END IF
            if (str1[5 ] != 'B' ){
                manobra[5 ] = 'B';
                manobra[6 ] = '0';
                manobra[7 ] = '.';
                manobra[8 ] = '0';
                manobra[9 ] = '0';
            } // END IF 
            if (str1[10] != 'C' ){
                manobra[10] = 'C';
                manobra[11] = '0';
                manobra[12] = '.';
                manobra[13] = '0';
                manobra[14] = '0';
            } // END IF
            if (str1[15] != 'D' ){
                manobra[15] = 'D';
                manobra[16] = '0';
                manobra[17] = '.';
                manobra[18] = '0';
                manobra[19] = '0';
            } // END IF
            if (str1[20] != 'E' ){
                manobra[20] = 'E';
                manobra[21] = '0';
                manobra[22] = '.';
                manobra[23] = '0';
                manobra[24] = '0';
            } // END IF
            if (str1[25] != 'F' ){
                manobra[25] = 'F';
                manobra[26] = '0';
                manobra[27] = '.';
                manobra[28] = '0';
                manobra[29] = '0';
            } // END IF
            if (str1[30] != 'G' ){
                manobra[30] = 'G';
                manobra[31] = '0';
                manobra[32] = '.';
                manobra[33] = '0';
                manobra[34] = '0';
            } // END IF
            if (str1[35] != 'H' ){
                manobra[35] = 'H';
                manobra[36] = '0';
                manobra[37] = '.';
                manobra[38] = '0';
                manobra[39] = '0';
            } // END IF
      for (size_t i = 0; manobra[i]!='\0'; i++) str1 [i] = manobra [i];
}// end Proto_485_V4
// =================================================================================

void Gerenciador_rede       (                                                   ){
    wdt_reset();
    String end_cliente = "P00X*";
                    //   1   2   3   4   5   6   7   8
    String msg_padrao = "A000B000C000D000E000F000G000H000";
    msg_padrao = str1;
    int tam_msg_485 = 41;
    char montar_msg[tam_msg_485];
    // conversão bruta //
    int posicao_inicial = 0,
        posicao_final = 0;
    int conta_caracter = 0;
     
        while (Serial.available()) {
            char request = Serial.read();
            str.concat(request);
            int tamanho_string = str.length();
            charRecebida[conta_caracter] = request;
            //Serial.println  ("AOBA: "         );
            //Serial.println  (request);

               if (charRecebida[conta_caracter]   == '\0' && conta_caracter > 5) {
                  //Serial.println  ("AOBA: "         );
                  int     final_msg    =  0, 
                          P_msg        =  0;
                     for (size_t i = 0;  charRecebida[i]!='\0'; i++){
                           if (charRecebida[i]   == 'P' && charRecebida[i+1]   == '0'  ) P_msg     = i;  
                           if (charRecebida[i]   == '*'                                ) final_msg = i+1;
                     }// END FOR
                  
                  for (size_t i = P_msg; i < P_msg+5; i++) Serial.print  (charRecebida[i]);

                  conta_caracter = 0;
                  str.remove(0, str.length());
                  for (size_t i = 0;  charRecebida[i]!='\0'; i++) charRecebida[i] = "";

                  break; // PARA O FOR NA MARRA!!!!
               }// END IF
            conta_caracter++;
         } // end while
         if (conta_caracter >45){
                Serial.print  ("\nAPAGOU: \n" );
                str.remove(0, str.length());
                for (size_t i = 0;  charRecebida[i]!='\0'; i++) charRecebida[i] = "";
                str = "";
                ////limpa string e char mesmo sendo temporaria////////
                conta_caracter = 0;
            }// end if   
}//void Gerenciador_rede()
// =================================================================================

void imprimir_serial_temp   (                                                   ){
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
                     vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                     break;
                  case 2:
                     pulso = pulsos_3;
                     vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                     break;
                  case 3:
                     pulso = pulsos_4;
                     vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                     break;
                  case 4:
                     pulso = pulsos_5;
                     vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                     break;
                  case 5:
                     pulso = pulsos_6;
                     vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                     break;
                  case 6:
                     pulso = pulsos_7;
                     vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                     break;
                  case 7:
                     pulso = pulsos_7;
                     vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                     break;
                  case 8:
                     pulso = pulsos_8;
                     vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
                     break;
                  case 9:
                     pulso = pulsos_9;
                     vetor_RPM(referencia, pulso, pulsos_por_volta, end_sensor);
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
         contador();
      }// end if  
}// end  imprimir_serial_temp
// =================================================================================

void vetor_RPM    (float ref_temp, int n_pulso, int n_pulso_volta, int end_sensor){
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

void simu_Sensor            (int temp                                           ){
         digitalWrite(MASTER, HIGH);
         delayMicroseconds(temp);
         digitalWrite(MASTER, LOW);
         delayMicroseconds(temp);    
}// end  simu_Sensor()
// =================================================================================

void Gerenciador_endereco   (                                                   ){
            // clinte com todos sensores definir endereço
            if (bit_3 == 0 && bit_2 == 0 && bit_1 == 0 && bit_0 == 1 ) end_cliente_num = 1;
            if (bit_3 == 0 && bit_2 == 0 && bit_1 == 1 && bit_0 == 0 ) end_cliente_num = 2;
            if (bit_3 == 0 && bit_2 == 0 && bit_1 == 1 && bit_0 == 1 ) end_cliente_num = 3;
            if (bit_3 == 0 && bit_2 == 1 && bit_1 == 0 && bit_0 == 0 ) end_cliente_num = 4;
            if (bit_3 == 0 && bit_2 == 1 && bit_1 == 0 && bit_0 == 1 ) end_cliente_num = 5;
            if (bit_3 == 0 && bit_2 == 1 && bit_1 == 1 && bit_0 == 0 ) end_cliente_num = 6;
            if (bit_3 == 0 && bit_2 == 1 && bit_1 == 1 && bit_0 == 1 ) end_cliente_num = 7;
            if (bit_3 == 1 && bit_2 == 0 && bit_1 == 0 && bit_0 == 0 ) end_cliente_num = 8;
            if (bit_3 == 1 && bit_2 == 0 && bit_1 == 0 && bit_0 == 1 ) end_cliente_num = 9;
            if (bit_3 == 1 && bit_2 == 0 && bit_1 == 1 && bit_0 == 0 ) end_cliente_num = 10;
            if (bit_3 == 1 && bit_2 == 0 && bit_1 == 1 && bit_0 == 1 ) end_cliente_num = 11;
            if (bit_3 == 1 && bit_2 == 1 && bit_1 == 0 && bit_0 == 0 ) end_cliente_num = 12;
            if (bit_3 == 1 && bit_2 == 1 && bit_1 == 0 && bit_0 == 1 ) end_cliente_num = 13;
            if (bit_3 == 1 && bit_2 == 1 && bit_1 == 1 && bit_0 == 0 ) end_cliente_num = 14;
            if (bit_3 == 1 && bit_2 == 1 && bit_1 == 1 && bit_0 == 1 ) end_cliente_num = 15;
            // DEBUG
            Serial.print  ("ENDERECO CONVERSOR: "  );
            Serial.println  (end_cliente_num         );
            switch (end_cliente_num)
                    { 
                        case  1:
                                end_CONF = "P001*";
                                break;
                        case  2:
                                end_CONF = "P002*";
                                break;
                        case  3:
                                end_CONF = "P003*";
                                break;
                        case  4:
                                end_CONF = "P004*";
                                break;
                        case  5:
                                end_CONF = "P005*";
                                break;
                        case  6:
                                end_CONF = "P006*";
                                break;
                        case  7:
                                end_CONF = "P007*";
                                break;
                        case  8:
                                end_CONF = "P008*";
                                break;
                        case  9:
                                end_CONF = "P009*";
                                break;
                        case  10:
                                end_CONF = "P010*";
                                break;
                        case  11:
                                end_CONF = "P011*";
                                break;
                        case  12:
                                end_CONF = "P012*";
                                break;
                        case  13:
                                end_CONF = "P013*";
                                break;
                        case  14:
                                end_CONF = "P014*";
                                break;
                        case  15:
                                end_CONF = "P015*";
                                break;
                        default:
                                wdt_reset();
                                break;
                    }// END SWITCH
}// end Gerenciador_endereco
// =================================================================================

int contador(){
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
   /////////////////////////////////////////
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
void Proto_485_V5                (){
      String end_cliente   = "P00X*";
      String comparador    = ""; //comparador.concat(charRecebida[i]);
      int final_msg = 0, P_msg = 0;
         while (Serial.available()) {
            char request = Serial.read();
            str.concat(request);
            int tamanho_string = str.length();
            charRecebida[conta_caracter] = request;
            conta_caracter++;
               for (size_t i = 0; charRecebida[i]!='\0'; i++)
                  {
                     if (charRecebida[i]=='P')   P_msg        = i;
                     if (charRecebida[i]=='*'){
                              final_msg   = i+1;
                              charRecebida[i+2]='\0';
                        }// END IF 
                  }// end for
               if (final_msg >= 4){
                        for (size_t i = 0; i < final_msg-P_msg; i++) {
                            comparador.concat(charRecebida[i]);
                           }// END FOR
                            
                           //Serial.println  (end_cliente);
                           /*
                          
                           RODAR AQUI A FUNÇÃO TACOMETRO
                           
                           */
                          if (comparador == end_CONF)
                          {
                              ciclos1++;  // CONTADOR DE QUANTAS INTERAÇÕES
                              tacometro  ( comparador);
                          }
 
                        for (size_t i = 0;  charRecebida[i]!='\0'; i++) charRecebida[i] = "";
                           final_msg = 0, P_msg = 0, conta_caracter = 0, comparador = "";
                  }// END IF
           // wdt_reset();
            
         } // end while
}// END PROTO_485_V5
// =================================================================================
void tacometro              ( String end_cliente                                ){       
// rotina rede 485
   if (millis() - timeold >= 500){
      int long timeold_2 = (millis() - timeold);
      //rpm = 60*1000 / t * actualREV => rpm = 60*1000 / (millis() - time ) * REV/2 OR rpm = 30*1000 / (millis() - time) * REV; //n = f . 60 / p
      int referencia =  60000 / timeold_2; //converte tempo de minutos para microsegundos
      float rpm      =  0.0,
            t_m      =  0.0;
      String temp_msg_padrao  = "";
      String temp_            = "";
      rpm = ( (float)(referencia) * (float)(pulsos_2) )/( (float)(pulsos_por_volta) );
      t_m = rpm;
            if (t_m<0) t_m = 0;
            if (t_m>999)temp_msg_padrao = String(t_m, 0);
            if (t_m <10)temp_msg_padrao = String(t_m, 2);
            if (t_m>=10)temp_msg_padrao = String(t_m, 1);
            ident_sensor_G = 1;
            Proto_485_V3(ident_sensor_G);  
            temp_.concat(str_sensor); 
            temp_.concat(temp_msg_padrao);
            temp_.toCharArray(str1, temp_.length()+1);
      rpm = 0;
      rpm = ( (float)(referencia) * (float)(pulsos_3) )/( (float)(pulsos_por_volta) );
      t_m = rpm;
            if (t_m<0) t_m = 0;
            if (t_m>999)temp_msg_padrao = String(t_m, 0);
            if (t_m <10)temp_msg_padrao = String(t_m, 2);
            if (t_m>=10)temp_msg_padrao = String(t_m, 1);
            ident_sensor_G = 2;
            Proto_485_V3(ident_sensor_G);  
            temp_.concat(str_sensor); 
            temp_.concat(temp_msg_padrao);
            temp_.toCharArray(str1, temp_.length()+1);  
      rpm = 0;
      rpm = ( (float)(referencia) * (float)(pulsos_4) )/( (float)(pulsos_por_volta) );
      t_m = rpm;
            if (t_m<0) t_m = 0;
            if (t_m>999)temp_msg_padrao = String(t_m, 0);
            if (t_m <10)temp_msg_padrao = String(t_m, 2);
            if (t_m>=10)temp_msg_padrao = String(t_m, 1);
            ident_sensor_G = 3;
            Proto_485_V3(ident_sensor_G);  
            temp_.concat(str_sensor); 
            temp_.concat(temp_msg_padrao);
            temp_.toCharArray(str1, temp_.length()+1);
      rpm = 0;
      rpm = ( (float)(referencia) * (float)(pulsos_5) )/( (float)(pulsos_por_volta) );
      t_m = rpm;
            if (t_m<0) t_m = 0;
            if (t_m>999)temp_msg_padrao = String(t_m, 0);
            if (t_m <10)temp_msg_padrao = String(t_m, 2);
            if (t_m>=10)temp_msg_padrao = String(t_m, 1);
            ident_sensor_G = 4;
            Proto_485_V3(ident_sensor_G);  
            temp_.concat(str_sensor); 
            temp_.concat(temp_msg_padrao);
            temp_.toCharArray(str1, temp_.length()+1);
      rpm = 0;
      rpm = ( (float)(referencia) * (float)(pulsos_6) )/( (float)(pulsos_por_volta) );
      t_m = rpm;
            if (t_m<0) t_m = 0;
            if (t_m>999)temp_msg_padrao = String(t_m, 0);
            if (t_m <10)temp_msg_padrao = String(t_m, 2);
            if (t_m>=10)temp_msg_padrao = String(t_m, 1);
            ident_sensor_G = 5;
            Proto_485_V3(ident_sensor_G);  
            temp_.concat(str_sensor); 
            temp_.concat(temp_msg_padrao);
            temp_.toCharArray(str1, temp_.length()+1);
      rpm = 0;
      rpm = ( (float)(referencia) * (float)(pulsos_7) )/( (float)(pulsos_por_volta) );
      t_m = rpm;
            if (t_m<0) t_m = 0;
            if (t_m>999)temp_msg_padrao = String(t_m, 0);
            if (t_m <10)temp_msg_padrao = String(t_m, 2);
            if (t_m>=10)temp_msg_padrao = String(t_m, 1);
            ident_sensor_G = 6;
            Proto_485_V3(ident_sensor_G);  
            temp_.concat(str_sensor); 
            temp_.concat(temp_msg_padrao);
            temp_.toCharArray(str1, temp_.length()+1);
      rpm = 0;
      rpm = ( (float)(referencia) * (float)(pulsos_8) )/( (float)(pulsos_por_volta) );
      t_m = rpm;
            if (t_m<0) t_m = 0;
            if (t_m>999)temp_msg_padrao = String(t_m, 0);
            if (t_m <10)temp_msg_padrao = String(t_m, 2);
            if (t_m>=10)temp_msg_padrao = String(t_m, 1);
            ident_sensor_G = 7;
            Proto_485_V3(ident_sensor_G);  
            temp_.concat(str_sensor); 
            temp_.concat(temp_msg_padrao);
            temp_.toCharArray(str1, temp_.length()+1);
      rpm = 0;
      rpm = ( (float)(referencia) * (float)(pulsos_9) )/( (float)(pulsos_por_volta) );
      t_m = rpm;
            if (t_m<0) t_m = 0;
            if (t_m>999)temp_msg_padrao = String(t_m, 0);
            if (t_m <10)temp_msg_padrao = String(t_m, 2);
            if (t_m>=10)temp_msg_padrao = String(t_m, 1);
            ident_sensor_G = 8;
            Proto_485_V3(ident_sensor_G);  
            temp_.concat(str_sensor); 
            temp_.concat(temp_msg_padrao);
            temp_.toCharArray(str1, temp_.length()+1);

         Proto_485_V4(str1,temp_.length()); // monta char com leituras 
         //DEBUG VETOR RPM REDE
            String msg = str1;
            Proto_485_V2(end_CONF, msg);
            //Serial.println(str1 );
         // FIM DEBUG
      rpm = 0;
      //Serial.print  (" Delta_T: "   );
      //Serial.print  (timeold_2/1000 );
      //Serial.println("s"            );
      pulsos_2=0,pulsos_3=0,pulsos_4=0,pulsos_5=0,pulsos_6=0,pulsos_7=0,pulsos_8=0,pulsos_9=0,rpm=0, timeold_2 = 0;
      timeold = millis();
      contador();
   }// end if
}// end  tacometro()
// =================================================================================

 





