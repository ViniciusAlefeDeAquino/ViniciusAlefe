void append_page_header() {
  webpage  = F("<!DOCTYPE html><html>");
  webpage += F("<head>");
  webpage += F("<title>IFMG - Drone Server</title>");
   webpage += F("<meta charset=utf-8>");
  webpage += F("<meta name='viewport' content='user-scalable=yes,initial-scale=1.0,width=device-width';charset=utf-8>");
  webpage += F("<style>");
  webpage += F("body{max-width:65%;margin:0 auto;font-family:arial;font-size:105%;text-align:center;color:blue;background-color:#E9E9E9;}");
  webpage += F("ul{list-style-type:none;margin:0.1em;padding:0;border-radius:0.375em;overflow:hidden;background-color:#90EE90;font-size:1em;align:center}");
  webpage += F("h5{color:white;padding:0;border-radius:0.375em;background-color:#228B22;font-size:1em;text-align:center}");
  webpage += F("formatar{color:white;padding:0.01em;border-radius:0.250em;background-color:#2E8B57;font-size:1em;text-align:center;text-decoration:none}");
  webpage += F("formatar a{color:white;text-decoration:none;padding:0.5em;border-radius:0.250em;background-color:#2E8B57;font-size:1em;text-align:center;}");
  webpage += F("formatar a:hover{color:#004400;text-decoration:none;padding:0.6em;border-radius:0.250em;background-color:#90EE90;font-size:1.1em;text-align:center;}");
  webpage += F("formatar a:active{color:#004400;text-decoration:none;padding:0.65em;border-radius:0.250em;background-color:#90EE90;font-size:1.1em;text-align:center;}");
  webpage += F("li{float:left;border-radius:0.375em;}last-child {border-right:none;font-size:85%;color:#004400;align:center;}");
  webpage += F("li a{display: block;border-radius:0.375em;padding:0.44em 0.44em;text-decoration:none;font-size:85%;color:#004400;text-align:center}");
  webpage += F("li a:hover{background-color:#006400;padding:0.48em 0.48em;border-radius:0.375em;font-size:97%;color:#ffffff;text-align:center}");
  webpage += F("li a:active{background-color:#006400;border-radius:0.375em;padding:0.50em 0.50em;font-size:97%;color:#00ff00;text-align:center}");
  webpage += F("section {font-size:0.88em;}");
  webpage += F("h1{color:white;border-radius:0.9em;font-size:1.3em;padding:0.2em 0.2em;background:#228B22;}");
  webpage += F("h2{color:#004400;font-size:1.0em;}");
  webpage += F("h3{font-size:1.0em;color:#004400;}");
  webpage += F("h4{font-size:1.0em;color:#004400;align:center;float:auto}");
  webpage += F("h6{font-size:0.8em;color:#004400;text-align: left;}");
  webpage += F("table{font-family:arial,sans-serif;font-size:0.9em;border-collapse:collapse;width:85%;}"); 
  webpage += F("th,td {border:0.06em solid #dddddd;text-align:left;padding:0.3em;border-bottom:0.06em solid #dddddd;color:#004400}");
  webpage += F(".texto {color:#004400;font-size:100%;text-align:Left}");
  webpage += F("tr:nth-child(odd) {background-color:#eeeeee;}");
  webpage += F(".rcorners_n {border-radius:0.5em;background:#2E8B57;padding:0.3em 0.3em;width:20%;color:white;font-size:75%;}");
  webpage += F(".rcorners_m {border-radius:0.5em;background:#2E8B57;padding:0.3em 0.3em;width:50%;color:white;font-size:75%;}");
  webpage += F(".rcorners_x {border-radius:0.5em;background:#2E8B57;padding:0.3em 0.3em;width:50%;color:white;font-size:95%;float:auto}");
  webpage += F(".rcorners_w {border-radius:0.5em;background:#2E8B57;padding:0.3em 0.3em;width:70%;color:white;font-size:75%;}");
  webpage += F(".column{float:left;width:50%;height:45%;}");
  webpage += F(".row:after{content:'';display:table;clear:both;}");
  webpage += F("*{box-sizing:border-box;}");
  webpage += F("footer{background-color:#eedfff; text-align:center;padding:0.3em 0.3em;border-radius:0.375em;font-size:60%;}");
  webpage += F("button{border-radius:0.5em;background:#2E8B57;padding:0.3em 0.3em;width:20%;color:white;font-size:130%;}");
  webpage += F(".buttons {border-radius:0.5em;background:#2E8B57;padding:0.3em 0.3em;width:15%;color:white;font-size:90%;}");
  webpage += F(".buttonsm{border-radius:0.5em;background:#2E8B57;padding:0.3em 0.3em;width:9%; color:white;font-size:70%;}");
  webpage += F(".buttonm {border-radius:0.5em;background:#2E8B57;padding:0.3em 0.3em;width:15%;color:white;font-size:70%;}");
  webpage += F(".buttonw {border-radius:0.5em;background:#2E8B57;padding:0.3em 0.3em;width:40%;color:white;font-size:70%;}");
  webpage += F("a{font-size:75%;}");
  webpage += F("p{font-size:75%;}");
  webpage += F("</style></head><body><h1> IFMG - Drone Server </h1>");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void append_page_menu(){ // Saves repeating many lines of code for HTML page footers
  webpage += F("<ul>");
  webpage += F("<li><a href='/'>Home</a></li>"); // Lower Menu bar command entries
  webpage += F("<li><a href='/download'>Baixar</a></li>"); 
  webpage += F("<li><a href='/upload'>Enviar</a></li>"); 
  webpage += F("<li><a href='/delete'>Apagar</a></li>"); 
  webpage += F("<li><a href='/dir'>Arquivos</a></li>");
  webpage += F("<li><a href='/calibrar'>Calibrar</a></li>");
  webpage += F("<li><a href='/test'>Teste</a></li>");
  webpage += F("</ul>");
}

void append_page_footer(){ // Saves repeating many lines of code for HTML page footers
  webpage += F("<br>");
  webpage += F("<br>");
  webpage += F("<h5>IFMG - Campus Betim");
  webpage += F("</h5>");
  webpage += F("</body></html>");
}

void append_page_formatar(){
  webpage += F("<br>");
  webpage += F("<formatar><a href='/format'>  Formatar  </a>");
  webpage += F("</formatar>");
}
void append_page_iniciar(){
  webpage += F("<br>");
  webpage += F("<formatar><a href='/start'>  Iniciar  </a>");
  webpage += F("</formatar>");
}
void append_page_calibrar(){
  webpage += F("<br>");
  webpage += F("<formatar><a href='/calibrado'>  Calibrar  </a>");
  webpage += F("</formatar>");
}
