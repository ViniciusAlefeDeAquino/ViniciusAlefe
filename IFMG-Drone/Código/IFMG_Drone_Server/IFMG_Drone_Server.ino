#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include "SPIFFS.h"
#include "FS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "FS.h"
String  webpage = "";
#include "CSS.h"
#include <SPI.h>

#define M1 0
#define M2 1                      //canais pwm para cado motor do drone
#define M3 2
#define M4 3


#define PW_MAX 750                //potencia máxima do PWM de 10 bits para limitar os motores. (0 a 750) 
#define TEST_PWM 5
#define FREQ 60                   //frequencia do PWM

#define B1 15
#define B2 4
#define B3 16                     //bits do sensor ultrassônico
#define B4 17
#define B5 5
#define B6 18

#define MEDIA_ULTRA 4           //média do sensor ultrssônico, distancia mínima e máxima.
#define DIST_MAX 43             
#define DIST_MIN 23           

#define TEMPO_AMOSTRAGEM 5000   //tempo de amostragem em microssegundos
#define I2C_CLOCK 400000        //Clock de comunicação I2C em Hz
#define MEDIA 1               //nº de elementos da média móvel dos sensores 
#define MEDIA_POS 1             //média móvel para os ângulos pós-processados
#define MEDIA_POT 75            //média móvel para leitura dos potenciômetros
#define TEMPO 4800              // tempo entre cada aquisição de dados do giroscópio para integração
#define LOOP_REGULADOR 10000     //número de ciclos para ajuste inicial do giroscópio
#define EXPOENTE 10

#define PERCENT_GYRO 0.995          //99,5%
#define MEDIA_ALT 10
#define MPx 0x68                //endereço do chip

LiquidCrystal_I2C lcd(0x27,16,2);
bool    SPIFFS_present = false;
bool comandoAtivo = false;
bool leituraCompleta = false;

int primeiroLoop = 1;
int bit_dist[6];
int buff[6];
int z_loop, contador_bits, decimal;
int maiorAcX, maiorAcY, maiorAcZ;
int menorAcX, menorAcY, menorAcZ;
int maiorGyX, maiorGyY, maiorGyZ;
int menorGyX, menorGyY, menorGyZ;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax, ay, az, gx, gy, gz, auxIntegral, altura;
float vaX[MEDIA], vaY[MEDIA], vaZ[MEDIA], vgX[MEDIA], vgY[MEDIA], vgZ[MEDIA];
float regulaGz, regulaGy, regulaGx;
float integralX[5], integralY[5], integralZ[5], rotX, rotY, rotZ, auxrot;
int i, tempo_a, tempo_b, tempo, j, a1, b1, tempo1[MEDIA_ALT], tempo_aux, tempo_acumulado, b, d;
float p1, p2, p3, p4, PEscrita_M1, PEscrita_M2, PEscrita_M3, PEscrita_M4, w1, w2, w3, w4;
float  p1_aux[MEDIA_POT],p2_aux[MEDIA_POT],p3_aux[MEDIA_POT],p4_aux[MEDIA_POT],p1_aux2,p2_aux2,p3_aux2,p4_aux2;
int primeiro = 1;
double angx, angy, d_rotX_dt, d_rotY_dt, d_rotZ_dt;
double AngX[MEDIA_POS], AngY[MEDIA_POS], AngX_aux, AngY_aux;
float operador;
float altitude = 0;

float sumAceleration, aux_sumAceleration;

float offsetN, offsetP, bug, debug, Px, Nx;
float offsetX, offsetY, offsetZ;

unsigned long tempo_ultra;
float tempo_ultra_aux;
float dist_aux;
float dist[MEDIA_ULTRA];
float distancia;
float range, tensao;
int i1;

int tempoComando = 0;
int numInt = 0;
float tempoEscrito;
int comandoManual=0;
int pararTeste=0;
int mfAl,mfRol,mfArf,mfGui;
int ctAl,ctRol,ctArf,ctGui;
float KpAl,KpRol,KpArf,KpGui;
float KiAl,KiRol,KiArf,KiGui;
float KdAl,KdRol,KdArf,KdGui;
float IntAl,IntRol,IntArf,IntGui;
float DerAl,DerRol,DerArf,DerGui;
float auxIntAl[2],auxIntRol[2],auxIntArf[2],auxIntGui[2];
float auxDerAl[2],auxDerRol[2],auxDerArf[2],auxDerGui[2];
float ErrAl,ErrRol,ErrArf,ErrGui;
float Al,Rol,Arf,Gui;


WebServer server(80);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup(void) {

  
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  
  Wire.begin();
  Wire.setClock(I2C_CLOCK);

  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);

  pinMode(B1, INPUT);
  pinMode(B2, INPUT);
  pinMode(B3, INPUT);
  pinMode(B4, INPUT);
  pinMode(B5, INPUT);
  pinMode(B6, INPUT);

  pinMode(34, INPUT);
  pinMode(33, INPUT);
  pinMode(32, INPUT);
  pinMode(35, INPUT);

  pinMode(1, INPUT);
  pinMode(3, INPUT);
  pinMode(14, INPUT);
  pinMode(25, INPUT);
  pinMode(36, INPUT);
  pinMode(39, INPUT);
  pinMode(23, INPUT);
  pinMode(19, INPUT);
  

  float time1,time2,timex;
  lcd.init(); 
  lcd.backlight();
  time1=micros();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Inicializando");
  time2=micros();
  lcd.setCursor(0,1);
  lcd.print("Aguarde 30s.");

  b = 1;
  d = 2;
  
  analogReadResolution(9);

  ledcAttachPin(12, M1);
  ledcSetup(M1, FREQ, 10);

  ledcAttachPin(13, M2);
  ledcSetup(M2, FREQ, 10);

  ledcAttachPin(26, M3);
  ledcSetup(M3, FREQ, 10);
  
  ledcAttachPin(27, M4);
  ledcSetup(M4, FREQ, 10);

  ledcWrite(M1,0);
  ledcWrite(M2,0);
  ledcWrite(M3,0);
  ledcWrite(M4,0);

  Serial.begin(250000);

  // Definindo o clock de comunicação I2C
  Wire.begin();
  Wire.setClock(I2C_CLOCK);
  Wire.beginTransmission(MPx);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  /*---------------------------------------------------*/

  //Definindo a escala do acelerômetro
  tempo_a = micros();
  Wire.beginTransmission(MPx);
  Wire.write(0x1C);                  // edereço do registrador
  Wire.write(0x0);                   // 0x0; 0x8; 0x10; 0x18: valores disponíveis em hexadecimal
  Wire.endTransmission(true);

  /*-------------------------------------------------*/
  //Definindo a escala do giroscópio

  Wire.beginTransmission(MPx);
  Wire.write(0x1B);                  // endereço do registrador
  Wire.write(0x0);                   // 0x0; 0x8; 0x10; 0x18: valores disponíveis em hexadecimal
  Wire.endTransmission(true);
  tempo_b = micros();
  tempo = tempo_b - tempo_a;
  Serial.println(tempo);
  Serial.println("");
  /*-------------------------------------------------*/

  RegulaGyro(NULL);              //Função que ajusta o giroscópio inicialmente

  /*-------------------------------------------------*/

  //Definindo o valor inicial das variaveis necessárias
  rotZ = 0;
  rotX = 0;
  rotY = 0;
  auxrot = 0;
  tempo_a = 0;
  tempo_b = 0;
  for (i = 0; i < MEDIA; i++) {
    vaX[i] = 0;
    vaY[i] = 0;
    vaZ[i] = 0;
    vgX[i] = 0;
    vgY[i] = 0;
    vgZ[i] = 0;
  }
  for (i = 0; i < MEDIA_POS; i++) {
    AngX[i] = 0;
    AngY[i] = 0;
  }
  for (i = 0; i < MEDIA_ALT; i++) {
    tempo1[i] = 0;
  }

  for (i = 0; i < MEDIA_ULTRA; i++) {
    dist[i] = 0;
  }

  integralX[0] = 0;
  integralX[1] = 0;
  integralY[0] = 0;
  integralY[1] = 0;
  integralZ[0] = 0;
  integralZ[1] = 0;
 /*-------------------------------------------------*/
  //conectando-se à rede wifi
  
  /*
    reconectar:
  i=0;
  WiFi.begin("rraaquino Oi Fibra (2.4G)", "rraaquino1980");
  while (WiFi.status() != WL_CONNECTED) {
    if(i>20){
      WiFi.disconnect();
      goto reconectar;
    }      
    i++;
    delay(250); Serial.print('.');
  }
  i=0;
  */

  
  delay(28000);                                                     //agurdando o roteador inicializar
  reconectar:
  i=0;
  WiFi.begin("IFMG-Drone", "");
  while (WiFi.status() != WL_CONNECTED) {
    if(i>20){
      WiFi.disconnect();
      goto reconectar;
    }      
    i++;
    delay(250); Serial.print('.');
  }
  i=0;
  
  
  Serial.println("\nEndereço IP: " + WiFi.localIP().toString());
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Rede: IFMG-Drone");
  lcd.setCursor(0,1);
  lcd.print("IP: ");
  lcd.setCursor(4,1);
  lcd.print(WiFi.localIP().toString());
  
   /*-------------------------------------------------*/
/*
  //gerando a própria rede wifi pelo esp32
  WiFi.softAP("IFMG - Drone Server", NULL, 5, 0, 4);
  Serial.println("\nEndereço IP: " + WiFi.softAPIP().toString());
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Endereco IP:");
    lcd.setCursor(0,1);
    lcd.print(WiFi.softAPIP().toString());
*/  

  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);
  delay(100);
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS não inicializado");
    SPIFFS_present = false;
  }
  else
  {
    Serial.println(F("SPIFFS iniciado"));
    SPIFFS_present = true;
  }
  //----------------------------------------------------------------------   Páginas do Servidor
  server.on("/",         HomePage);
  server.on("/download", File_Download);
  server.on("/upload",   File_Upload);
  server.on("/fupload",  HTTP_POST, []() {
    server.send(200);
  }, handleFileUpload);
  server.on("/delete",   File_Delete);
  server.on("/dir",      SPIFFS_dir);
  server.on("/start",      Start);
  server.on("/test",      Test);
  server.on("/format",      Format);
  server.on("/calibrar",      Calibrar);
  server.on("/calibrado",      Calibrado);

  server.begin();
  Serial.println("Servidor HTTP Iniciado");

  disableCore0WDT();
  disableCore1WDT();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop(void) {

  server.handleClient(); // Servidos Ouvindo o cliente
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void HomePage(){
  SendHTML_Header();
  append_page_menu();
  webpage += F("<h3>Apresentação<p></h3>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Esta é uma planta didática construída para estudos de modelagem e aplicação de controle em sistemas dinâmicos. Nela, é possível a atuação em malha aberta ou fechada em até 3 diferentes sistemas (arfagem, rolagem e guinada), cada um deles representado um ângulo de posicionamento do drone em funcionamento.<p></h6>");
  webpage += F("<h3>Como utilizar<p></h3>");
  webpage += F("<h7><p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b>Montagem física</b></p></h7>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Para utilizar a planta e atuar em seus sistemas, é necessário, antes de tudo, verificar o correto posicionamento do drone em planta. Verifique se todos os conectores estão conectados e devidamente posicionados. Certifique-se tambem que o elástico de fixação está bem esticado.</h6>");
  webpage += F("<h7><p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b>Calibragem</b></p></h7>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Com o drone devidamente posicionado e a planta corretamente ligada, é necessária a realização do procedimento de calibragem dos sensores do drone. As instuções para calibrar os sensores podem ser encontradas na página de calibragem. A calibragem sempre deve ser realizada uma vez após a planta ser ligada, podendo ser feita facultativamente outras vezes durante a utilização.</h6>");
  webpage += F("<h7><p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b>Envio de comandos</b></p></h7>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Outro passo necessário é o envio do arquivo de comando para a planta. Esse arquivo será o responsável tanto pela definição dos parâmetros e controladores do teste, como dos valores de referência a serem seguidos pelos sistemas em cada passo de tempo. Dessa forma, é possível a aplicação de sinais com formas de onda variadas como diferentes amplitudes para cada um dos sistemas presente na planta. O nome do arquivo de comando enviado deve ser exclusivamente \"comandos.csv\" (formato csv). Mais instruções para o envio de comandos podem ser acessados na página 'enviar'. Exemplos e o gabarito do arquivo de comando podem ser encontrados a partir do link:<a href='https://github.com/ViniciusAlefeDeAquino/ViniciusAlefe/tree/master/IFMG-Drone'> https://github.com/ViniciusAlefeDeAquino/ViniciusAlefe/tree/master/IFMG-Drone.</a></h6>");
  webpage += F("<h7><p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b>Iniciar</b></p></h7>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Após a calibragem dos sensores e o envio do arquivo de comando, os ensaios dinâmicos podem ser realizados através da página 'Teste'. Na página 'Teste' é realizada a execução dos comandos enviados à planta. Mais instruções sobre a execução dos ensaios dinâmicos podem ser encontrados nessa mesma página.<p></h6>");
  webpage += F("<h7><p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b>Resultados</b></p></h7>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Após a conclusão dos testes, a planta gera um arquivo de resultados no formato 'csv' com todos os resultados dos sensores em função do tempo durante o teste. É possível baixar o arquivo de resultados na página 'Baixar'.</h6>");
  webpage += F("<h7><p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b>Outros recursos</b></p></h7>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Em caso de má utilização dos recursos de download e upload de arquivos a planta pode apresentar problemas. É possível observar o diretório de arquivos da planta através da página 'Arquivos' e, caso ocorra algum problema, é possível formatar o diretório de arquivos na mesma página. Não recomenda-se o upload de arquivos que não sejam de comando, uma vez que arquivos alheios à utilização da planta serão perdidos durante ensaios dinâmicos.</h6>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Ao solicitar o envio de arquivos, arquivos já armazenados com o mesmo nome serão substituídos, entretanto, é possivel ainda excluir arquivos através da página 'Apagar'.</h6>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Em caso de dúvidas ou problemas, o desenvolvedor dessa planta pode ser contactado pelo email: viniracim@gmail.com.</h6>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Vinicius Alefe de Aquino - ECA - 6ºPeríodo, IFMG-Betim (2019.2)</h6>");
  webpage += F("<h7><p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b></b></p></h7>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectado");
  lcd.setCursor(0,1);
  lcd.print("Home page");

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Download() {
  if (server.args() > 0 ) {
    if (server.hasArg("download")) DownloadFile(server.arg(0));
  }
  else SelectInput("Digite o nome do arquivo para baixar", "download", "download");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectado");
  lcd.setCursor(0,1);
  lcd.print("Donload");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void DownloadFile(String filename) {
  if (SPIFFS_present) {
    File download = SPIFFS.open("/" + filename,  "r");
    if (download) {
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else ReportFileNotPresent("download");
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Upload() {

  append_page_header();
  append_page_menu();
  webpage += F("<h3>Enviar arquivo de comando</h3>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Para uma melhor compreensão do correto formato de arquivos para comandos da planta, recomenda-se vizualizar os arquivos exeplares e o gabarito de comandos disponíveis no link:<a href='https://github.com/ViniciusAlefeDeAquino/ViniciusAlefe/tree/master/IFMG-Drone'> https://github.com/ViniciusAlefeDeAquino/ViniciusAlefe/tree/master/IFMG-Drone.</a></h6>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Para iniciar a utilização e compreender melhor o funcionamento geral da planta recomenda-se a utilização dos arquivos exemplares como comando. Para pessoas que não têm o domínio da dinâmica dos sistemas disponíveis (rolagem, arfagem e guinada) recomenda-se a utilização primeiramente da rolagem (segunda coluna no arquivo de comando, como consta no gabarito) como sistema em malha aberta. Normalmente a rolagem, em malha aberta, tem uma resposta semelhante a de um sistema dinâmico linear de segunda ordem.</h6>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Atenção! Todo arquivo de comando enviado deve estar nomeado como \"comandos.csv\", e somente assim. Caso contrário o software não encontrará os comandos enviados. Logo, para utilizar os comandos de exemplo disponíveis no github, renomeie os arquivos para o nome correto caso estejam com um nome diferente.</h6>");
  webpage += F("<h7><p>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b>Potenciômetros</b></p></h7>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Em caso de seleção de comando manual no arquivo enviado, os potenciômetros representam respectivamente (da esquerda para a direita):</h6>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;1 - Potência média geral dos motores (altitude). O software ajusta automaticamente o valor mínimo necessário acordo com a entrada aplicada para valores de arfagem, rolagem ou guinada, não sendo necessária a utilização durante ensaios. Recomenda-se repousar em zero. Os valores do potenciômetro variam de 0 a 127. Ponto zero na marcação inicial.</h6>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;2 - Rolagem. Varia de -80 a 80 em malha fechada e de -20 a 20 em malha aberta. Ponto zero na marcação mediana.</h6>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;3 - Arfagem. Varia de -80 a 80 em malha fechada e de -20 a 20 em malha aberta. Ponto zero na marcação mediana.</h6>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;4 - Guinada. Varia de -127 a 127 em qualquer configuração. Ponto zero na marcação mediana.</h6>");
  webpage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
  webpage += F("<input class='buttons' style='width:40%' type='file' name='fupload' id = 'fupload' value=''><br>");
  webpage += F("<br><button class='buttons' style='width:10%' type='submit'>Enviar Arquivo</button><br>");
  webpage += F("<a href='/'></a><br><br>");
  append_page_footer();
  server.send(200, "text/html", webpage);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectado");
  lcd.setCursor(0,1);
  lcd.print("Enviar Comando");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
File UploadFile;
void handleFileUpload() {

  HTTPUpload& uploadfile = server.upload();

  if (uploadfile.status == UPLOAD_FILE_START)
  {
    String filename = uploadfile.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.print("Nome do arquivo: "); Serial.println(filename);
    UploadFile = SPIFFS.open(filename, "w");  //Cria o arquivo
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
    if (UploadFile) UploadFile.write(uploadfile.buf, uploadfile.currentSize); // Escrevendo os dados no arquivo
  }
  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if (UploadFile)         // Se o arquivo foi criado corretamente
    {
      UploadFile.close();
      Serial.print("Upload Size: "); Serial.println(uploadfile.totalSize);
      webpage = "";
      append_page_header();
      append_page_menu();
      webpage += F("<h3>Arquivo enviado com sucesso</h3>");
      webpage += F("<h2>Nome do arquivo: "); webpage += uploadfile.filename + "</h2>";
      webpage += F("<h2>Tamanho: "); webpage += file_size(uploadfile.totalSize) + "</h2><br>";
      append_page_footer();
      server.send(200, "text/html", webpage);

    }
    else
    {
      ReportCouldNotCreateFile("upload");
    }
  }

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_dir() {
  if (SPIFFS_present) {
    File root = SPIFFS.open("/");
    if (root) {
      root.rewindDirectory();
      SendHTML_Header();
      append_page_menu();
      webpage += F("<h3>Diretório de Arquivos</h3>");
      webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Aqui podem ser visualizados todos os arquivos disponíveis na memória interna do dispositivo. Em caso de problemas no diretório (a não vizualização de arquivos, por exemplo), o botão \"formatar\" pode ser acionado como solução.</h6>");
      webpage += F("<h3 class='rcorners_x'>Conteúdo Interno - Esp32</h3><br>");
      webpage += F("<table align='center'>");
      webpage += F("<tr><th>Nome</th><th style='width:20%'>Tipo</th><th>Tamanho</th></tr>");
      printDirectory("/", 0);
      webpage += F("</table>");
      

      int bytes = SPIFFS.totalBytes();
      String fsize = "";
      if (bytes < 1024)                     fsize = String(bytes) + " B";
      else if (bytes < (1024 * 1024))        fsize = String(bytes / 1024.0, 3) + " KB";
      else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
      else                                  fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";

      webpage += "<p class='texto'> Espaço Total: " + fsize + "<br>";

      bytes = SPIFFS.totalBytes() - SPIFFS.usedBytes();
      fsize = "";
      if (bytes < 1024)                     fsize = String(bytes) + " B";
      else if (bytes < (1024 * 1024))        fsize = String(bytes / 1024.0, 3) + " KB";
      else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
      else                                  fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";

      webpage += "Espaço Livre: " + fsize + "</p>";

      append_page_formatar();
      append_page_footer();
      SendHTML_Content();
      root.close();
    }
    else
    {
      SendHTML_Header();
      webpage += F("<h3>Arquivos não foram encontrados</h3>");
    }

    SendHTML_Content();
    SendHTML_Stop();
  } else ReportSPIFFSNotPresent();
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectado");
  lcd.setCursor(0,1);
  lcd.print("Arquivos");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void printDirectory(const char * dirname, uint8_t levels) {
  String tiraBarra;             //variável para tirar a '/' que tm no início da string String(file.name())
  File root = SPIFFS.open(dirname);
  if (!root) {
    return;
  }
  if (!root.isDirectory()) {
    return;
  }
  File file = root.openNextFile();
  while (file) {
    if (webpage.length() > 1000) {
      SendHTML_Content();
    }
    if (file.isDirectory()) {
      webpage += "<tr><td>" + String(file.isDirectory() ? "Dir" : "File") + "</td><td>" + String(file.name()) + "</td><td></td></tr>";
      printDirectory(file.name(), levels - 1);
    }
    else
    {
      tiraBarra = String(file.name());
      tiraBarra[0] = ' ';                   //Tirando a barra do início do nome
      webpage += "<tr><td>" + tiraBarra + "</td>";
      webpage += "<td>" + String(file.isDirectory() ? "Dir" : "File") + "</td>";
      int bytes = file.size();
      String fsize = "";
      if (bytes < 1024)                     fsize = String(bytes) + " B";
      else if (bytes < (1024 * 1024))        fsize = String(bytes / 1024.0, 3) + " KB";
      else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
      else                                  fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";
      webpage += "<td>" + fsize + "</td></tr>";
    }
    file = root.openNextFile();
  }
  file.close();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Delete() {
  if (server.args() > 0 ) {
    if (server.hasArg("delete")) SPIFFS_file_delete(server.arg(0));
  }
  else SelectInput("Selecione um arquivo para apagar", "delete", "delete");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectado");
  lcd.setCursor(0,1);
  lcd.print("Deletar Arquivo");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_file_delete(String filename) {
  if (SPIFFS_present) {
    SendHTML_Header();
    append_page_menu();
    File dataFile = SPIFFS.open("/" + filename, "r");
    if (dataFile)
    {
      if (SPIFFS.remove("/" + filename)) {
        Serial.println(F("Arquivo apagado com sucesso"));
        webpage += "<h3>O arquivo '" + filename + "' foi apagado</h3>";
        webpage += F("<a href='/delete'></a>");
      }
      else
      {
        webpage += F("<h3>Arquivo não encontrado</h3>");
        webpage += F("<a href='delete'></a>");
      }
    } else ReportFileNotPresent("delete");
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();
  } else ReportSPIFFSNotPresent();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectado");
  lcd.setCursor(0,1);
  lcd.print("Arquivo Deletado");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Header() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  append_page_header();
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Content() {
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Stop() {
  server.sendContent("");
  server.client().stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SelectInput(String heading1, String command, String arg_calling_name) {
  SendHTML_Header();
  append_page_menu();
  webpage += F("<h3>"); webpage += heading1 + "</h3>";
  webpage += F("<FORM action='/"); webpage += command + "' method='post'>";
  webpage += F("<input type='text' name='"); webpage += arg_calling_name; webpage += F("' value=''><br>");
  webpage += F("<type='submit' name='"); webpage += arg_calling_name; webpage += F("' value=''><br><br>");
  webpage += F("<a href='/'></a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportSPIFFSNotPresent() {
  SendHTML_Header();
  append_page_menu();
  webpage += F("<h3>SPIFFS não detectado</h3>");
  webpage += F("<a href='/'></a><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportFileNotPresent(String target) {
  SendHTML_Header();
  append_page_menu();
  webpage += F("<h3>O Arquivo não existe</h3>");
  webpage += F("<a href='/"); webpage += target + "'></a>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportCouldNotCreateFile(String target) {
  SendHTML_Header();
  append_page_menu();
  webpage += F("<h3>Não foi possível enviar o arquivo</h3>");
  webpage += F("<a href='/"); webpage += target + "'></a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
String file_size(int bytes) {
  String fsize = "";
  if (bytes < 1024)                 fsize = String(bytes) + " B";
  else if (bytes < (1024 * 1024))      fsize = String(bytes / 1024.0, 3) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
  else                              fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";
  return fsize;
}

void Format() {
  append_page_header();
  append_page_menu();
  if (SPIFFS.format()) {
    webpage += F("<h3>Dispositivo formatado com sucesso</h3>");
    Serial.printf("Dispositivo formatado com sucesso");
  }
  else {
    webpage += F("<h3>ERRO NA FORMATAÇÃO</h3>");
    Serial.printf("ERRO NA FORMATAÇÃO");
  }
  append_page_footer();
  server.send(200, "text/html", webpage);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectado");
  lcd.setCursor(0,1);
  lcd.print("Formatado");
}

void Test() {
  append_page_header();
  append_page_menu();
  webpage += F("<h3>Iniciar Teste</h3>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Acione o botão \"Iniciar\" para que a planta execute os comandos enviados. Caso selecionado o comando manual via potenciômetros no arquivo de comando, recomenda-se posicionar os potenciômetros nos pontos nulos marcados fisicamente na planta. As descrições dos potenciômetros encontram-se na página \"Enviar\".</h6>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Em caso de problemas severos de leitura dos dados, o teste será automaticamente encerrado para garantir a integridade dos dados do arquivo de resultados. Testes seguros duram por até pouco mais de 1 minuto, assim, testes maiores que isso podem ser encerrados devido ao exesso de espaço ocupado na memória ou instabilidades de leituras dos sensores, sendo imediatamente encerrados após a ocorrência de problemas (um teste abortado não descarta o arquivo de resultados gerado).</h6>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Após a conclusão do processo, será gerado o arquivo \"resultados.csv\" que pode ser baixado na página \"Baixar\".</h6>");
  append_page_iniciar();
  append_page_footer();
  server.send(200, "text/html", webpage);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectado");
  lcd.setCursor(0,1);
  lcd.print("Iniciar");
}

void Calibrar() {
  append_page_header();
  append_page_menu();
  webpage += F("<h3>Calibragem de sensores</h3>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Mantenha o drone completamente estático e acione o botão \"calibrar\". Mantenha o drone parado até que o procedimento de calibragem seja concluído.</h6>");
  append_page_calibrar();
  append_page_footer();
  server.send(200, "text/html", webpage);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectado");
  lcd.setCursor(0,1);
  lcd.print("Calibragem");
}

void Calibrado() {
  RegulaGyro(NULL);
  append_page_header();
  append_page_menu();
  webpage += F("<h3>Sensores calibrados com sucesso</h3>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Agora, o drone está pronto para ser operado.</h6>");
  append_page_footer();
  server.send(200, "text/html", webpage);
  digitalWrite(2,HIGH);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectado");
  lcd.setCursor(0,1);
  lcd.print("Calibrado");
  delay(85);
  digitalWrite(2, LOW);
  delay(100);
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);
}
void Start() {
  primeiroLoop = 1;
  Serial.println("Começou o teste");
  append_page_header();
  append_page_menu();
  webpage += F("<h3>Teste iniciado com sucesso, aguarde.</h3>");
  webpage += F("<h6>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;O sistema fará a leitura dos comandos e, em aproximadamente 15 segundos, os comandos serão iniciados. Esta webpage permanecerá não operacional até que a execução de todos comandos seja concluída.</h6>");
  append_page_footer();
  server.send(200, "text/html", webpage);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Preparando");
  lcd.setCursor(0,1);
  lcd.print("Agurade 15s");

  if (!SPIFFS.exists("/comandos.csv")) {
    Serial.println("Não há arquivo de comando. Teste encerrado");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("ERRO");
    lcd.setCursor(0,1);
    lcd.print("Comando ausente");
    return;
  }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Preparando");
  lcd.setCursor(0,1);
  lcd.print("Agurade 15s");

  Go(NULL);

  digitalWrite(2, LOW);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conectado");
  lcd.setCursor(0,1);
  lcd.print("Teste Concluido");
  Serial.println("Terminou o teste");
}

void Go(void *parametro) {
  float tAmostraInicial, tAmostraAtual;
  float tempoInicial, tempoAtual;
  float tempoEscritoAnterior = 0;



  xTaskCreatePinnedToCore(Comando, "Comando", 90000, NULL, 0, NULL, 0);

  while (leituraCompleta == false) {
    delayMicroseconds(3);
  }

  File resultados = SPIFFS.open("/resultados.csv", FILE_WRITE);
  resultados.println("tempo(us);arfagem;rolagem;guinada;altitude;p1;p2;p3;p4");


  auxrot = 0;
  tempo_a = 0;
  tempo_b = 0;
  for (i = 0; i < MEDIA; i++) {
    vaX[i] = 0;
    vaY[i] = 0;
    vaZ[i] = 0;
    vgX[i] = 0;
    vgY[i] = 0;
    vgZ[i] = 0;
  }
  for (i = 0; i < MEDIA_POS; i++) {
    AngX[i] = 0;
    AngY[i] = 0;
  }
  for (i = 0; i < MEDIA_ALT; i++) {
    tempo1[i] = 0;
  }

  for (i = 0; i < MEDIA_ULTRA; i++) {
    dist[i] = 0;
  }


  while (comandoAtivo == false) {
    delayMicroseconds(3);
  }

  float tempoTeste = tempoComando * numInt;

  rotZ = 0;
  rotX = 0;
  rotY = 0;
  integralX[0]=0;
  integralY[0]=0;
  integralZ[0]=0;
  integralX[1]=0;
  integralY[1]=0;
  integralZ[1]=0;

  IntAl = 0;
  IntRol = 0;
  IntArf = 0;
  IntGui = 0;
  auxIntAl[0] = 0;
  auxIntRol[0] = 0;
  auxIntArf[0] = 0;
  auxIntGui[0] = 0;
  auxIntAl[1] = 0;
  auxIntRol[1] = 0;
  auxIntArf[1] = 0;
  auxIntGui[1] = 0;
  auxDerAl[0] = 0;
  auxDerRol[0] = 0;
  auxDerArf[0] = 0;
  auxDerGui[0] = 0;
  auxDerAl[1] = 0;
  auxDerRol[1] = 0;
  auxDerArf[1] = 0;
  auxDerGui[1] = 0;
  

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~LOOP DE LEITURA DOS SENSORES
  tempoInicial = micros();
  tAmostraInicial = micros();
  tempo_a = micros();
  for (;;) {

    for (z_loop = 0; z_loop < 3; z_loop++) {
      ax = 0;
      ay = 0;
      az = 0;
      gx = 0;
      gy = 0;
      gz = 0;
      AngX_aux = 0;
      AngY_aux = 0;
      altitude = 0;

      
      for (contador_bits = 0; contador_bits < 6; contador_bits++) {
        bit_dist[contador_bits] = 0;
      }
      /*-------------------------------------------------*/
      //LENDO AS VARIÁVEIS DE TODOS OS SENSORES

      tempoAtual = micros();
      
      Wire.beginTransmission(MPx);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPx, 14, true);

      AcX = Wire.read() << 8 | Wire.read();
      AcY = Wire.read() << 8 | Wire.read();
      AcZ = Wire.read() << 8 | Wire.read();
      Tmp = Wire.read() << 8 | Wire.read();
      GyX = Wire.read() << 8 | Wire.read();
      GyY = Wire.read() << 8 | Wire.read();
      GyZ = Wire.read() << 8 | Wire.read();

      //-----------------------------------------------------------------------
      //ACUMULAÇÃO DA MÉDIA MÓVEL UTILIZADA

      if(primeiroLoop==1){
        for (i = 0; i < MEDIA; i++) {
          vaX[i] = AcX;
          vaY[i] = AcY;
          vaZ[i] = AcZ;
          vgX[i] = GyX;
          vgY[i] = GyY;
          vgZ[i] = GyZ;
        }
      }

      for (i = 0; i < MEDIA - 1; i++) {
        vaX[i] = vaX[i + 1];
        vaY[i] = vaY[i + 1];
        vaZ[i] = vaZ[i + 1];
        vgX[i] = vgX[i + 1];
        vgY[i] = vgY[i + 1];
        vgZ[i] = vgZ[i + 1];

        ax += vaX[i];
        ay += vaY[i];
        az += vaZ[i];
        gx += vgX[i];
        gy += vgY[i];
        gz += vgZ[i];
      }


      vaX[MEDIA - 1] = AcX;
      vaY[MEDIA - 1] = AcY;
      vaZ[MEDIA - 1] = AcZ;
      vgX[MEDIA - 1] = GyX;
      vgY[MEDIA - 1] = GyY;
      vgZ[MEDIA - 1] = GyZ;

      ax += vaX[MEDIA - 1];
      ay += vaY[MEDIA - 1];
      az += vaZ[MEDIA - 1];
      gx += vgX[MEDIA - 1];
      gy += vgY[MEDIA - 1];
      gz += vgZ[MEDIA - 1];
      //-----------------------------------------------------------------------


      //-----------------------------------------------------------------------
      //REGULAGEM INICIAL DOS VALORES LIDOS PELOS SESORES

      ax = ax / (MEDIA);
      ay = ay / (MEDIA);
      az = az / (MEDIA);
      gx = gx / (MEDIA);
      gy = gy / (MEDIA);
      gz = gz / (MEDIA);

      ax += 3020;
      ay -= 1786;
      az -= 654;

      ax = -ax;

      ay = ay / 63.7382;
      ax = ax / 62.1056;
      az = az / 67.6215;

      ax = ax / 259.5;
      ay = ay / 259.1;
      az = az / 249.5;

      gx -= regulaGx;
      gy -= regulaGy;
      gz -= regulaGz;

      d_rotX_dt = gx / 130;

      d_rotY_dt = gy / 130;

      d_rotZ_dt = gz * 0.00761905;
      //-----------------------------------------------------------------------

      if (digitalRead(B1) == HIGH)
        bit_dist[0] = 1;
      if (digitalRead(B2) == HIGH)
        bit_dist[1] = 1;
      if (digitalRead(B3) == HIGH)
        bit_dist[2] = 1;
      if (digitalRead(B4) == HIGH)
        bit_dist[3] = 1;
      if (digitalRead(B5) == HIGH)
        bit_dist[4] = 1;
      if (digitalRead(B6) == HIGH)
        bit_dist[5] = 1;

      decimal = (32 * bit_dist[0]) + (16 * bit_dist[1]) + (8 * bit_dist[2]) + (4 * bit_dist[3]) + (2 * bit_dist[4]) + bit_dist[5];

      if (decimal == 63)
        altitude = 0;
      else {
        altitude = (DIST_MAX - DIST_MIN);
        altitude = (altitude * decimal) / 60;
        altitude += DIST_MIN;
      }

      //-----------------------------------------------------------------------
      //OBTENDO OS ÂNGULOS
      GetAngle(ax, ay, az);      //função que obtém as inclinações a paM4ir do acelerômetro
      sumAceleration = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
      //-----------------------------------------------------------------------

      tempo_b = micros();
      tempo = tempo_b - tempo_a;
      tempo_a = micros();

      integralX[0] = integralX[1];
      integralX[1] = d_rotX_dt;
      rotX += (tempo * 0.000001) * 0.5 * (integralX[0] + integralX[1]);

      
      integralY[0] = integralY[1];
      integralY[1] = d_rotY_dt;
      rotY += (tempo * 0.000001) * 0.5 * (integralY[0] + integralY[1]);

      integralZ[0] = integralZ[1];
      integralZ[1] = d_rotZ_dt;
      rotZ += (tempo * 0.000001) * 0.5 * (integralZ[0] + integralZ[1]);
      //-----------------------------------------------------------------------

      /*
        if(sumAceleration <= 1){
        if(primeiroLoop==1){
          rotX = angy;
          rotY = angx;
        }
        aux_sumAceleration = pow((sumAceleration),EXPOENTE);

        angx =  aux_sumAceleration*rotY + (1 - aux_sumAceleration)*angx;
        angy =  aux_sumAceleration*rotX + (1 - aux_sumAceleration)*angy;

        rotX = (1 - aux_sumAceleration)*rotX +  aux_sumAceleration*angy;
        rotY = (1 - aux_sumAceleration)*rotY +  aux_sumAceleration*angx;
        }
        else{
        if(primeiroLoop==1){
          rotX = angy;
          rotY = angx;
        }
        aux_sumAceleration = pow((2-sumAceleration),EXPOENTE);

        angx =  aux_sumAceleration*rotY + (1 - aux_sumAceleration)*angx;
        angy =  aux_sumAceleration*rotX + (1 - aux_sumAceleration)*angy;

        rotX = (1 - aux_sumAceleration)*rotX +  aux_sumAceleration*angy;
        rotY = (1 - aux_sumAceleration)*rotY +  aux_sumAceleration*angx;
        }
      */
      
      if (primeiroLoop == 1) {
        rotX = angy;
        rotY = angx;
      }
      

      angx = (1 - PERCENT_GYRO) * angx + PERCENT_GYRO * rotY;
      angy = (1 - PERCENT_GYRO) * angy + PERCENT_GYRO * rotX;

      rotX = angy;
      rotY = angx;

      //-----------------------------------------------------------------------
      //media móvel final
      
      if(primeiroLoop==1){
        for (i = 0; i < MEDIA_POS; i++) {
          AngX[i] = angx;
          AngY[i] = angy;
        }
      }
      
      for (i = 0; i < MEDIA_POS - 1; i++) {
        AngX[i] = AngX[i + 1];
        AngY[i] = AngY[i + 1];

        AngX_aux += AngX[i];
        AngY_aux += AngY[i];
      }

      AngX[MEDIA_POS - 1] = angx;
      AngY[MEDIA_POS - 1] = angy;

      AngX_aux += AngX[MEDIA_POS - 1];
      AngY_aux += AngY[MEDIA_POS - 1];

      AngX_aux = AngX_aux / (MEDIA_POS);
      AngY_aux = AngY_aux / (MEDIA_POS);
      //-----------------------------------------------------------------------
      
      ErrAl= PEscrita_M1 - altitude;
      ErrRol= PEscrita_M2 - AngX_aux;
      ErrArf= PEscrita_M3 - AngY_aux;
      ErrGui= PEscrita_M4 - rotZ;

      //-----------------------------------------------------------------------
      
      auxIntAl[0] = auxIntAl[1];
      auxIntAl[1] = ErrAl;
      IntAl += (tempo * 0.000001) * 0.5 * (auxIntAl[0] + auxIntAl[1]);

      auxIntRol[0] = auxIntRol[1];
      auxIntRol[1] = ErrRol;
      IntRol += (tempo * 0.000001) * 0.5 * (auxIntRol[0] + auxIntRol[1]);
      
      auxIntArf[0] = auxIntArf[1];
      auxIntArf[1] = ErrArf;
      IntArf += (tempo * 0.000001) * 0.5 * (auxIntArf[0] + auxIntArf[1]);

      auxIntGui[0] = auxIntGui[1];
      auxIntGui[1] = ErrGui;
      IntGui += (tempo * 0.000001) * 0.5 * (auxIntGui[0] + auxIntGui[1]);

      //-----------------------------------------------------------------------

      auxDerAl[0] = auxDerAl[1];
      auxDerAl[1]= ErrAl;
      DerAl=(auxDerAl[1]-auxDerAl[0])/(tempo * 0.000001);

      auxDerRol[0] = auxDerRol[1];
      auxDerRol[1]= ErrRol;
      DerRol=(auxDerRol[1]-auxDerRol[0])/(tempo * 0.000001);

      auxDerArf[0] = auxDerArf[1];
      auxDerArf[1]= ErrArf;
      DerArf=(auxDerArf[1]-auxDerArf[0])/(tempo * 0.000001);

      auxDerGui[0] = auxDerGui[1];
      auxDerGui[1]= ErrGui;
      DerGui=(auxDerGui[1]-auxDerGui[0])/(tempo * 0.000001);
      
      
      //-----------------------------------------------------------------------
      Al = PEscrita_M1;
      if(mfAl==1){
        Al=ErrAl;
        if(ctAl==1){
          Al= KpAl*ErrAl+KiAl*IntAl+KdAl*DerAl; 
        }
      }

      Rol = PEscrita_M2;
      if(mfRol==1){
        Rol=ErrRol;
        if(ctRol==1){
          Rol= KpRol*ErrRol+KiRol*IntRol+KdRol*DerRol; 
        }
      }      

      Arf = PEscrita_M3;
      if(mfArf==1){
        Arf=ErrArf;
        if(ctArf==1){
          Arf= KpArf*ErrArf+KiArf*IntArf+KdArf*DerArf; 
        }
      }
      
      Gui = PEscrita_M4;
      if(mfGui==1){
        Gui=ErrGui;
        if(ctGui==1){
          Gui= KpGui*ErrGui+KiGui*IntGui+KdGui*DerGui; 
        }
      }
      
      //-----------------------------------------------------------------------
      //Gravando os dados no arquivo

      if (z_loop == 0) {
        tempoEscrito = tempoAtual - tempoInicial;

        resultados.printf("%.0f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f", tempoEscrito, AngY_aux, AngX_aux, rotZ, altitude, PEscrita_M1, PEscrita_M2, PEscrita_M3, PEscrita_M4);
        resultados.println("");
        tempoEscritoAnterior = tempoEscrito;

        if (comandoAtivo == false)
          break;
      }

      if (tempo > 80000 && primeiroLoop != 1){
        Serial.println(tempo);
        pararTeste = 1;
      }
 

      if (comandoAtivo == false)
        break;
      //-----------------------------------------------------------------------
      //Mostrando os dados pelo monitor serial
      
        //Serial.print("X = "); Serial.print(ax);
       // Serial.print(" | Y = "); Serial.print(ay);
        //Serial.print(" | Z = "); Serial.println(az);
      //-----------------------------------------------------------------------
      primeiroLoop = 0;
    }

    if (tempoEscrito > tempoTeste || comandoAtivo == false)
      break;
  }

  resultados.close();
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void Comando(void *parametro) {

  digitalWrite(2, HIGH);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(2, LOW);
  
  File comandos = SPIFFS.open("/comandos.csv", "r");
  int vfSize;

  int tempoTeste = 0;
  int contador = 0;

  char buff;
  String numero = "";
  float tempoInicial, tempoAtual;
  float pMotor1, pMotor2, pMotor3, pMotor4, m1, m2, m3, m4;

  for (;;) {
    vfSize = comandos.position();
    buff = comandos.read();
    if (buff == '\n')
      numInt++;
    if (comandos.position() == vfSize)
      break;
  }
  numInt=numInt-7;
  comandos.close();

  if (numInt > 20000 || numInt < 1) {
    comandoAtivo = true;
    delayMicroseconds(100);
    comandoAtivo = false;
    vTaskDelete(NULL);
    return;
  }

  uint8_t motor1[numInt];
  uint8_t motor2[numInt];
  uint8_t motor3[numInt];
  uint8_t motor4[numInt];

  comandos = SPIFFS.open("/comandos.csv", "r");


  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',')
      break;
    numero += String(buff);
  }
  tempoComando = numero.toInt();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',')
      break;
    numero += String(buff);
  }
  comandoManual = numero.toInt();
  numero = "";


  for (;;) {
    buff = comandos.read();
    if ( buff == '\n')
      break;
  }
    for (;;) {
    buff = comandos.read();
    if ( buff == '\n')
      break;
  }
  
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  mfAl = numero.toInt();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  mfRol = numero.toInt();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  mfArf = numero.toInt();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  mfGui = numero.toInt();
  numero = "";


  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  ctAl = numero.toInt();
  numero = ""; 
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  ctRol = numero.toInt();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  ctArf = numero.toInt();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  ctGui = numero.toInt();
  numero = "";



  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KpAl = numero.toFloat(); 
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KpRol = numero.toFloat();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KpArf = numero.toFloat();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KpGui = numero.toFloat();
  numero = "";


  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KiAl = numero.toFloat(); 
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KiRol = numero.toFloat();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KiArf = numero.toFloat();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KiGui = numero.toFloat();
  numero = "";


  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KdAl = numero.toFloat(); 
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KdRol = numero.toFloat();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KdArf = numero.toFloat();
  numero = "";
  for (;;) {
    buff = comandos.read();
    if (buff == ';' || buff == ',' || buff == '\n')
      break;
    numero += String(buff);
  }
  KdGui = numero.toFloat();
  numero = "";
  
  for (;;) {
    buff = comandos.read();
    if ( buff == '\n')
      break;
  }

  for (contador = 0; contador < numInt; contador++) {
    if (comandos.position() == vfSize)
      break;
    numero = "";
    for (;;) {
      buff = comandos.read();
      if (buff == ';' || buff == ',' || buff == '\n')
        break;
      numero += String(buff);
    }

    motor1[contador] = numero.toInt() + 127;
    if (motor1[contador] > 254)
      motor1[contador] = 254;
    numero = "";

    for (;;) {
      buff = comandos.read();
      if (buff == ';' || buff == ',' || buff == '\n')
        break;
      numero += String(buff);
    }

    motor2[contador] = numero.toInt() + 127;
    if (motor2[contador] > 254)
      motor2[contador] = 254;
    numero = "";

    for (;;) {
      buff = comandos.read();
      if (buff == ';' || buff == ',' || buff == '\n')
        break;
      numero += String(buff);
    }

    motor3[contador] = numero.toInt() + 127;
    if (motor3[contador] > 254)
      motor3[contador] = 254;
    numero = "";

    for (;;) {
      buff = comandos.read();
      if (buff == ';' || buff == ',' || buff == '\n')
        break;
      numero += String(buff);
      if (comandos.position() == vfSize)
        break;
    }

    motor4[contador] = numero.toInt() + 127;
    if (motor4[contador] > 254)
      motor4[contador] = 254;
    numero = "";

  }
  comandos.close();

  SPIFFS.format();
  comandos = SPIFFS.open("/comandos.csv", FILE_WRITE);
  comandos.printf("%i;%i;;",tempoComando,comandoManual);
  comandos.println("");
  comandos.println(";;;");
  comandos.printf("%i;%i;%i;%i",mfAl,mfRol,mfArf,mfGui);
  comandos.println("");
  comandos.printf("%i;%i;%i;%i",ctAl,ctRol,ctArf,ctGui);
  comandos.println("");
  comandos.printf("%.3f;%.3f;%.3f;%.3f",KpAl,KpRol,KpArf,KpGui);
  comandos.println("");
  comandos.printf("%.3f;%.3f;%.3f;%.3f",KiAl,KiRol,KiArf,KiGui);
  comandos.println("");
  comandos.printf("%.3f;%.3f;%.3f;%.3f",KdAl,KdRol,KdArf,KdGui);
  comandos.println("");
  comandos.println(";;;");
  
  for (contador = 0; contador < numInt; contador++) {
    comandos.printf("%i;%i;%i;%i",motor1[contador]-127,motor2[contador]-127,motor3[contador]-127,motor4[contador]-127);
    if(contador!=numInt-1)
      comandos.println("");
  }
  comandos.close();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Conctado");
  lcd.setCursor(0,1);
  lcd.print("Executando");
  numero = "";
  leituraCompleta = true;
  digitalWrite(2, HIGH);

  delay(10);
  digitalWrite(2, LOW);
  delay(100);
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);


  int primeiroComando=1;

  if (comandoManual == 0) {
    comandoAtivo = true;
    for (contador = 0; contador < numInt; contador++) {
      tempoInicial = micros();

      m1 = motor1[contador] - 127;
      m2 = motor2[contador] - 127;
      m3 = motor3[contador] - 127;
      m4 = motor4[contador] - 127;

      PEscrita_M1 = m1;
      PEscrita_M2 = m2;
      PEscrita_M3 = m3;
      PEscrita_M4 = m4;

      if(primeiroComando==1){
        Al=PEscrita_M1;
        Rol=PEscrita_M2;
        Arf=PEscrita_M3;
        Gui=PEscrita_M4;
      }

      while(true){

        m1=Al;
        m2=Rol;
        m3=Arf;
        m4=Gui;

        if((m1*d)<(-d*m2-d*m3-b*m4))
          m1=(-d*m2-d*m3-b*m4)/d;
        if((m1*d)<(d*m2-d*m3+b*m4))
          m1=(d*m2-d*m3+b*m4)/d;
        if((m1*d)<(-d*m2+d*m3+b*m4))
          m1=(-d*m2+d*m3+b*m4)/d;
        if((m1*d)<(+d*m2+d*m3-b*m4))
          m1=(d*m2+d*m3-b*m4)/d;
          
        
        w1 = sqrt((d * m1 + d * m2 + d * m3 + b * m4) / (4 * b * d));
        w2 = sqrt((d * m1 - d * m2 + d * m3 - b * m4) / (4 * b * d));
        w3 = sqrt((d * m1 + d * m2 - d * m3 - b * m4) / (4 * b * d));
        w4 = sqrt((d * m1 - d * m2 - d * m3 + b * m4) / (4 * b * d));
  
        pMotor1 = (w1 * PW_MAX) / 5.6457;
        pMotor2 = (w2 * PW_MAX) / 5.6457;
        pMotor3 = (w3 * PW_MAX) / 5.6457;
        pMotor4 = (w4 * PW_MAX) / 5.6457;

        if(pMotor1>PW_MAX)
          pMotor1 = PW_MAX;
        if(pMotor2>PW_MAX)
          pMotor2 = PW_MAX;
        if(pMotor3>PW_MAX)
          pMotor3 = PW_MAX;
        if(pMotor4>PW_MAX)
          pMotor4 = PW_MAX;

          Serial.printf("%.1f  %.1f   %.1f  %.1f   ||   %.0f  %.0f   %.0f  %.0f\n",m1,m2,m3,m4,pMotor1,pMotor2,pMotor3,pMotor4);
  
        ledcWrite(M1, pMotor1);
        ledcWrite(M2, pMotor2);
        ledcWrite(M3, pMotor3);
        ledcWrite(M4, pMotor4);
        primeiroComando=0;
        tempoAtual = micros();
        if ((tempoAtual - tempoInicial + 5) > tempoComando || pararTeste==1)
          break;
      }
    }
  }
  else {
    tempoTeste = numInt * tempoComando;
    comandoAtivo = true;
    tempoInicial = micros();
    
    for (;;) {
      p1_aux2=0;
      p2_aux2=0;
      p3_aux2=0;
      p4_aux2=0;
      
      p1 = analogRead(34);
      p2 = analogRead(33);
      p3 = analogRead(32);
      p4 = analogRead(35);
      


      if(primeiroComando==1){
        for (j = 0; j < MEDIA_POT; j++) {
          p1_aux[j]=p1;
          p2_aux[j]=p2;
          p3_aux[j]=p3;
          p4_aux[j]=p4;
        }
      }
      for (j = 0; j < MEDIA_POT - 1; j++) {
        p1_aux[j] = p1_aux[j + 1];
        p2_aux[j] = p2_aux[j + 1];
        p3_aux[j] = p3_aux[j + 1];
        p4_aux[j] = p4_aux[j + 1];

        p1_aux2 += p1_aux[j];
        p2_aux2 += p2_aux[j];
        p3_aux2 += p3_aux[j];
        p4_aux2 += p4_aux[j];       
      }

      p1_aux[MEDIA_POT - 1] = p1;
      p2_aux[MEDIA_POT - 1] = p2;
      p3_aux[MEDIA_POT - 1] = p3;
      p4_aux[MEDIA_POT - 1] = p4;

      p1_aux2 += p1_aux[MEDIA_POT - 1];
      p2_aux2 += p2_aux[MEDIA_POT - 1];
      p3_aux2 += p3_aux[MEDIA_POT - 1];
      p4_aux2 += p4_aux[MEDIA_POT - 1];

      p1 = p1_aux2 / (MEDIA_POT);
      p2 = p2_aux2 / (MEDIA_POT);
      p3 = p3_aux2 / (MEDIA_POT);
      p4 = p4_aux2 / (MEDIA_POT);
      
      p1=(p1/511)*127;
      
      if(mfRol==1)
        p2=((p2/511)*160) - 80;
      else
        p2=((p2/511)*40) - 20;
        
      if(mfArf==1)
        p3=((p3/511)*160) - 80;
      else
        p3=((p3/511)*100) - 50;
        
        p4=((p4/511)*255) - 127;


      PEscrita_M1=p1;
      PEscrita_M2=p2;
      PEscrita_M3=p3;
      PEscrita_M4=p4;

      if(primeiroComando==1){
        Al=PEscrita_M1;
        Rol=PEscrita_M2;
        Arf=PEscrita_M3;
        Gui=PEscrita_M4;
      }

      p1=Al;
      p2=Rol;
      p3=Arf;
      p4=Gui;

      if((p1*d)<(-d*p2-d*p3-b*p4))
        p1=(-d*p2-d*p3-b*p4)/d;
      if((p1*d)<(d*p2-d*p3+b*p4))
        p1=(d*p2-d*p3+b*p4)/d;
      if((p1*d)<(-d*p2+d*p3+b*p4))
        p1=(-d*p2+d*p3+b*p4)/d;
      if((p1*d)<(+d*p2+d*p3-b*p4))
        p1=(d*p2+d*p3-b*p4)/d;
        
      w1 = sqrt((d * p1 + d * p2 + d * p3 + b * p4) / (4 * b * d));
      w2 = sqrt((d * p1 - d * p2 + d * p3 - b * p4) / (4 * b * d));
      w3 = sqrt((d * p1 + d * p2 - d * p3 - b * p4) / (4 * b * d));
      w4 = sqrt((d * p1 - d * p2 - d * p3 + b * p4) / (4 * b * d));

      pMotor1 = (w1 * PW_MAX) / 5.6457;
      pMotor2 = (w2 * PW_MAX) / 5.6457;
      pMotor3 = (w3 * PW_MAX) / 5.6457;
      pMotor4 = (w4 * PW_MAX) / 5.6457;

      if(pMotor1>PW_MAX)
        pMotor1 = PW_MAX;
      if(pMotor2>PW_MAX)
        pMotor2 = PW_MAX;
      if(pMotor3>PW_MAX)
        pMotor3 = PW_MAX;
      if(pMotor4>PW_MAX)
        pMotor4 = PW_MAX;

      ledcWrite(M1, pMotor1);
      ledcWrite(M2, pMotor2);
      ledcWrite(M3, pMotor3);
      ledcWrite(M4, pMotor4);
      primeiroComando=0;
      
      Serial.printf("%.0f ||  %.0f  ||  %.0f || %.0f\n", PEscrita_M1, PEscrita_M2, PEscrita_M3, PEscrita_M4);

      tempoAtual = micros();

      if ((tempoAtual - tempoInicial) >= tempoTeste ||  pararTeste == 1)
        break;
    }
  }
  pararTeste=0;
  ledcWrite(M1, 0);
  ledcWrite(M2, 0);
  ledcWrite(M3, 0);
  ledcWrite(M4, 0);

  numInt = NULL;
  comandoAtivo = false;
  leituraCompleta = false;
  delayMicroseconds(5);
  leituraCompleta = false;
  comandoAtivo = false;
  delayMicroseconds(5);
  comandoAtivo = false;
  Serial.println("TERMINOU COMANDO");

  digitalWrite(2, HIGH);
  vTaskDelay(pdMS_TO_TICKS(130));
  digitalWrite(2, LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(2, HIGH);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(2, LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(2, HIGH);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(2, LOW);
  
  vTaskDelete(NULL);
}

void RegulaGyro(void *parametro) {

  offsetX = 0;
  offsetY = 0;
  offsetZ = 0;

  Wire.beginTransmission(MPx);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  /*---------------------------------------------------*/

  //Definindo a escala do acelerômetro
  Wire.beginTransmission(MPx);
  Wire.write(0x1C);                  // edereço do registrador
  Wire.write(0x0);                   // 0x0; 0x8; 0x10; 0x18: valores disponíveis em hexadecimal
  Wire.endTransmission(true);

  /*-------------------------------------------------*/
  //Definindo a escala do giroscópio

  Wire.beginTransmission(MPx);
  Wire.write(0x1B);                  // endereço do registrador
  Wire.write(0x0);                   // 0x0; 0x8; 0x10; 0x18: valores disponíveis em hexadecimal
  Wire.endTransmission(true);

  gx = 0;
  gy = 0;
  gz = 0;

  for (i = 0; i < LOOP_REGULADOR; i++) {
    Wire.beginTransmission(MPx);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPx, 14, true);

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    gx += GyX;
    gy += GyY;
    gz += GyZ;
  }

  regulaGx = gx / LOOP_REGULADOR;
  regulaGy = gy / LOOP_REGULADOR;
  regulaGz = gz / LOOP_REGULADOR;

}

void GetAngle(float ax, float ay, float az) {
  if (ax >= 1.0)
    ax = 1.0;
  if (ay >= 1.0)
    ay = 1.0;

  if (ax <= -1.0)
    ax = -1.0;
  if (ay <= -1.0)
    ay = -1.0;

  if (az > 0.0) {
    angx = asin(ax);
  }
  else {
    angx = 3.14159265 - asin(ax);
  }

  if (az > 0.0) {
    angy = asin(ay);
  }
  else {
    if (angx >= 1.57079632) {
      angy = asin(ay);
    }
    else {
      angy = 3.14159265 - asin(ay);
    }
  }

  angx = (angx / 3.14159265) * 180;
  angy = (angy / 3.14159265) * 180;
}
