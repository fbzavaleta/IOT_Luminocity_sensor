/**
 * Lib C
 */
#include <stdio.h>
#include <string.h>
/**
 * FreeRTOS
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
/**
 * WiFi
 */
#include "esp_wifi.h"
/**
 * WiFi Callback
 */
#include "esp_event_loop.h"
/**
 * Log
 */
#include "esp_system.h"
#include "esp_log.h"
/**
 * NVS
 */
#include "nvs_flash.h"
/**
 * LWIP
 */
#include "lwip/err.h"
#include "lwip/sys.h"
#include <lwip/sockets.h>
/**
 * Erros
 */
#include <errno.h>

/**
 * Definições
 */
#define DEBUG 		1
#define SERVER_IP "184.106.153.149"
#define SERVER_PORT 80

#define SENSOR	GPIO_NUM_17
#define BLINK_GPIO 5 //led building

#define EXAMPLE_ESP_WIFI_SSID 	"benjamin32"
#define EXAMPLE_ESP_WIFI_PASS 	"5d728e$E"

/**
 * Variáveis
 */
QueueHandle_t Queue_Sensor;
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static const char *TAG = "main: ";

typedef struct xData {
 	int sock; 
 	uint32_t counter; 
} xSocket_t; 

/**
 * Protótipos
 */
void Task_Socket ( void * pvParameter );
void Task_SendReceive ( void * pvParameter );


/*CONFIGURAÇÃO DO WIFI-------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------*/

/**
 * Notificaçãoes durante a etapa de conexão
 */

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
		case SYSTEM_EVENT_STA_START:
			/*
				O WiFi do ESP32 foi configurado com sucesso. 
				Agora precisamos conectar a rede WiFi local. Portanto, foi chamado a função esp_wifi_connect();
			*/
			esp_wifi_connect();
			break;
		case SYSTEM_EVENT_STA_GOT_IP:
		    /*
				Se chegou aqui é porque a rede WiFi foi aceita pelo roteador e um IP foi 
				recebido e gravado no ESP32.
			*/
			ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
			/*
			   O bit WiFi_CONNECTED_BIT é setado pelo EventGroup pois precisamos avisar as demais Tasks
			   que a rede WiFi foi devidamente configurada. 
			*/
			xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
			break;
		case SYSTEM_EVENT_STA_DISCONNECTED:
		    /*
				Se chegou aqui foi devido a falha de conexão com a rede WiFi.
				Por esse motivo, haverá uma nova tentativa de conexão WiFi pelo ESP32.
			*/
			esp_wifi_connect();
			
			/*
				É necessário apagar o bit WIFI_CONNECTED_BIT para avisar as demais Tasks que a conexão WiFi
				está offline no momento. 
			*/
			xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
			break;
		default:
			break;
    }
    return ESP_OK;
}

/**
 * Inicio da conexão wifi
 */

void wifi_init_sta( void )
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

/**
 * Task de socket
 */
void Task_Socket ( void * pvParameter ) 
{
    int rc; 
	uint32_t counter; 
	xSocket_t xSocket;
	
	if( DEBUG )
		ESP_LOGI(TAG, "Task_Socket run ...\r\n");
	
    for(;;) 
    {
		/**
		 * Aguarda o Button ser pressionado.
		 */
		xQueueReceive( Queue_Sensor, &counter, portMAX_DELAY ); 
		

		int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		
		if( DEBUG )
			ESP_LOGI(TAG, "socket: rc: %d", sock);
		
		struct sockaddr_in serverAddress;
		serverAddress.sin_family = AF_INET;
		
		/**
		 * Registra o endereço IP e PORTA do servidor;
		 */
		inet_pton(AF_INET, SERVER_IP, &serverAddress.sin_addr.s_addr);
		serverAddress.sin_port = htons(SERVER_PORT);

		/**
		 * Tenta realiza a conexão socket com o servidor; 
		 * Caso a conexão ocorra com sucesso, será retornado 0, caso contrário, -1;
		 */
		rc = connect(sock, (struct sockaddr *)&serverAddress, sizeof(struct sockaddr_in));
		
		if( DEBUG )
			ESP_LOGI(TAG, "Status Socket: %d", rc);
		
		if( rc == -1 ) 
		{
			if( DEBUG )
				ESP_LOGI(TAG, "xiii Socket Error: %d", sock);

			/**
			 * Aguarda 5 segundos antes de abrir um novo socket;
			 */
			for( int i = 1; i <= 5; ++i )
			{
				if( DEBUG )
					ESP_LOGI(TAG, "timeout: %d", 5-i);

				vTaskDelay( 1000/portTICK_PERIOD_MS );
			}
			continue; 
		} 

		/**
		 * Se chegou aqui é porque o ESP32 está conectado com sucesso com o servidor Web.
		 * Portanto, é possível criar uma Task para enviar e receber dados com o servidor.
		 * O legal deste programa é que cada vez que button for pressionado uma nova task/thread
		 * será criada;
		 */
		xSocket.sock = sock; 
		xSocket.counter = counter; 
		
	    xTaskCreate( Task_SendReceive, "TaskSendReceive", 10000, (void*)&(xSocket), 5, NULL );
		
	}

	vTaskDelete( NULL );

}

/*ENVIO E RECEPÇÃO DE DADOS COM O SERVIDOR-----------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------*/

/**
 * Task Responsável em enviar e receber dados com o servidor web;
 */
void Task_SendReceive ( void * pvParameter ) 
{
	int rec_offset = 0; 
	/**
	 * Para armazenar os bytes recebidos do servidor Web 
	 * vou utilizar um buffer de 1024 bytes criado dinamicamente;
	 */
	int total =	1*1024; 
	char *buffer = pvPortMalloc( total );
	if( buffer == NULL ) 
	{
		if( DEBUG )
			ESP_LOGI(TAG, "pvPortMalloc Error\r\n");
		vTaskDelete(NULL); 	  
		return;
	 }
	 
	/**
	 * Recebe o Socket da conexão com o servidor web;
	 */
    xSocket_t * xSocket = (xSocket_t*) pvParameter;
	
	const char * msg_post = \

        "POST /update HTTP/1.1\n"
        "Host: api.thingspeak.com\n"
        "Connection: close\n"
        "X-THINGSPEAKAPIKEY: 63DLEJY8YTS71OCY\n"
        "Content-Type: application/x-www-form-urlencoded\n"
        "content-length: ";
		
	char databody[50];
  sprintf( databody, "{63DLEJY8YTS71OCY&field1=%d}", xSocket->counter);//&key=63DLEJY8YTS71OCY xSocket->counter 
	sprintf( buffer , "%s%d\r\n\r\n%s\r\n\r\n", msg_post, strlen(databody), databody);
  ESP_LOGI(TAG, "VALOR ENVIADO: %d", xSocket->counter);
  
	/****************************************************
	 * OK, agora é necessário enviar o cabeçalho HTTP para o servidor Web; 
	 * Lembre-se que já estamos conectados ao servidor Web. 
	 */

	int rc = send( xSocket->sock, buffer, strlen(buffer), 0 );

	if( DEBUG )
		ESP_LOGI(TAG, "Cabecalho HTTP Enviado? rc: %d", rc);
	
	for(;;)
	{
		/* sizeRead armazena a quantidade de bytes recebidos e armazenados em buffer */
		ssize_t sizeRead = recv(xSocket->sock, buffer+rec_offset, total-rec_offset, 0);
		
		/**
		 * Error durante a leitura. Devemos encerrar o socket; 
		 */
		if ( sizeRead == -1 ) 
		{
			if( DEBUG )
				ESP_LOGI( TAG, "recv: %d", sizeRead );
			break;
		}
		/**
		 * Servidor Web encerrou a conexão com o ESP32; Devemos encerrar o socket; 
		 */
		if ( sizeRead == 0 ) 
		{
			break;
		}
		
		/**
		 * Imprime na saída do console a resposta em Json do servidor Web;
		 * Lembre-se que os pacotes podem chegar fragmentados. Isso é do TCP! 
		 * Portanto, é necessário utilizar uma variável de offset para ir sempre 
		 * populando o buffer a partir de determinados indices;
		 */
		if( sizeRead > 0 ) 
		{	
			if( DEBUG ) 
		    	ESP_LOGI(TAG, "Socket: %d - Data read (size: %d) was: %.*s", xSocket->sock, sizeRead, sizeRead, buffer);
		   
		   rec_offset += sizeRead; 
		 }
        
        /**
         * É preciso dar oportunidade para o scheduler trocar o contexto e
         * chamar outras tasks;
         */
		vTaskDelay( 5/portTICK_PERIOD_MS );
	}
	
	/**
	 * Se chegou aqui é porque o servidor web desconectou o client;
	 * Portanto, fecha a conexão socket, libera o buffer da memória e deleta a task;
	 */
	rc = close(xSocket->sock);
	
	if( DEBUG )
		ESP_LOGI(TAG, "close: rc: %d", rc); 
	
	vPortFree( buffer );	

	vTaskDelete(NULL); 
}

/*CAPTURA DE DADOS DO SENSOR-------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------*/

void Task_Sensor ( void *pvParameter )
{
    int aux=0;
	uint32_t counter = 0;
		
    /* Configura a GPIO 17 como saída */ 
  	gpio_pad_select_gpio( SENSOR );	
    gpio_set_direction( SENSOR, GPIO_MODE_INPUT ); 

	if( DEBUG )
    	ESP_LOGI(TAG, "Leitura do Sensor\n");
    
    for(;;) 
	{
	
		if(gpio_get_level( SENSOR ) == 0 && aux == 0)
		{ 
		  /* Delay para tratamento do bounce da tecla*/
		  vTaskDelay( 1000/portTICK_PERIOD_MS );	
		  
		  if(gpio_get_level( SENSOR ) == 0) 
		  {	
			  counter = 1;

      if( xQueueSend(Queue_Sensor, &counter, (10/portTICK_PERIOD_MS)  == pdPASS ) )
			{
			   ESP_LOGI( TAG, "O VALOR %d DE COUNT FOI ENVIADO NA QUEUE.\n", counter );
			}	
			
			//aux = 1; 
		  }
		}

		if(gpio_get_level( SENSOR ) == 1 && aux == 0)
		{
		    vTaskDelay( 1000/portTICK_PERIOD_MS );	
			if(gpio_get_level( SENSOR ) == 1 )
			{
				counter = 0;
				if( xQueueSend(Queue_Sensor, &counter, (10/portTICK_PERIOD_MS)  == pdPASS ) )
				{
					 ESP_LOGI( TAG, "O VALOR %d DE COUNT FOI ENVIADO NA QUEUE.\n", counter );
				}
				aux = 0;
				
			}
		}	

		vTaskDelay( 10/portTICK_PERIOD_MS );	
    }
}


/**
Task de blink
*/
void task_personal1_blink(void * pvParameter) //parmetro de tipo void pointer(se debe hacer lel cast), obligatorio
{

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
	//loop Infinito	
	for(;;)
	{
        /* Liga Led */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Delisga Led */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        ESP_LOGI( TAG, "El Led ha sido parpadeado.\r\n");
	}
    
    
	
	vTaskDelete( NULL ); // caso se salga del loop Infinito(parametro de control), NULL = Autobloqueo de la Task
}

/*TASK MAIN------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------*/
void app_main( void )
{	
    /*
		Inicialização da memória não volátil para armazenamento de dados (Non-volatile storage (NVS)).
		**Necessário para realização da calibração do PHY. 
	*/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
	
	/*
	   Event Group do FreeRTOS. 
	   Só podemos enviar ou ler alguma informação TCP quando a rede WiFi estiver configurada, ou seja, 
	   somente após o aceite de conexão e a liberação do IP pelo roteador da rede (nos casos de IPs dinâmicos).
	*/
	wifi_event_group = xEventGroupCreate();

	/**
	 * Queue button é responsável em armazenar o valor do contador "count" incrementado
	 * cada vez que button for pressionado. O valor de count será enviado via Json POST
	 * para o servidor Web;
	 */
	Queue_Sensor = xQueueCreate( 5, sizeof(uint32_t) );

	/*
	  Configura a rede WiFi em modo station;
	*/
    wifi_init_sta();
	
	/**
	 * Aguarda o ESP32 se conectar a rede WiFi local.
	 */
	xEventGroupWaitBits( wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY );	
	
	/*
	   Task responsável em ler e enviar valores via Socket TCP Client. 
	*/
	if( xTaskCreate( Task_Sensor, "TaskButton", 4098, NULL, 5, NULL )!= pdTRUE )
    {
      if( DEBUG )
        ESP_LOGI( TAG, "error - Nao foi possivel alocar Task_Sensor.\r\n" );  
      return;   
    }
	if( xTaskCreate( Task_Socket, "TaskSocket", 10000, NULL, 5, NULL )!= pdTRUE )
    {
      if( DEBUG )
        ESP_LOGI( TAG, "error - Nao foi possivel alocar Task_Socket.\r\n" );  
      return;   
    }
	
    if( xTaskCreate( task_personal1_blink, "task_blink", 10000, NULL, 2, NULL ) != pdTRUE ) //mofdificar puntero de la funcion
    {

      ESP_LOGI( TAG, "error - Nao foi possivel alocar task_personal1_blink.\r\n" );  
      return;   
    }
}


