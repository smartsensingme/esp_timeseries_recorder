# Gravador de Séries Temporais para ESP

O `esp_timeseries_recorder` armazena séries temporais uniformemente amostradas
como registros intercalados de inteiros de 16 bits com sinal em um buffer
contíguo e estaticamente reservado na DRAM interna. Ele foi projetado para
aplicações ESP-IDF determinísticas: o caminho de gravação não aloca memória,
não produz logs, não inicia tarefas e não realiza operações de entrada e saída.

O componente é intencionalmente limitado à aquisição e ao armazenamento. USB,
sistemas de arquivos, CRCs, gráficos, sensores e algoritmos de controle
pertencem a outros componentes ou à aplicação.

## Singleton, propriedade e concorrência

O gravador é um **singleton**: o componente possui uma configuração global, um
buffer e uma máquina de estados, portanto suas funções não recebem um handle de
instância. Ele permite capturas sequenciais, mas não capturas independentes e
simultâneas.

Use um único contexto produtor para `esp_timeseries_record_f32()` e
`esp_timeseries_record_i16()`. Tarefas consumidoras podem consultar o estado e
o status a qualquer momento. Elas somente podem acessar as amostras por meio de
`esp_timeseries_get_capture()` enquanto o gravador permanecer no estado `FULL`.
Uma chamada a `esp_timeseries_clear()` invalida todas as visualizações da
captura obtidas anteriormente.

`esp_timeseries_init()` faz uma cópia superficial da configuração. Portanto, o
vetor de canais e suas strings devem permanecer válidos durante toda a execução
do programa; normalmente eles devem ser declarados com armazenamento estático.

## Configuração

As opções definidas em tempo de compilação são:

- `CONFIG_ESP_TIMESERIES_RECORDER_BUFFER_KIB`: quantidade de DRAM interna
  reservada para o buffer de amostras;
- `CONFIG_ESP_TIMESERIES_RECORDER_MAX_CHANNELS`: limite superior usado para os
  contadores de amostras inválidas ou saturadas de cada canal.

Durante a execução, `esp_timeseries_config_t` define:

- `producer_rate_hz`: frequência com que a aplicação chama uma função de
  gravação;
- `channel_count`: quantidade de valores em cada registro;
- `channels`: descritores na ordem exata utilizada em todas as amostras
  fornecidas.

Cada `esp_timeseries_channel_t` fornece `name`, `unit`, `scale`, `offset` e um
campo opcional `encoding`. Para a API de ponto flutuante, a conversão é:

```text
raw = round((real_value - offset) / scale)
real_value = raw * scale + offset
```

Por exemplo, `scale = 0.1` e `offset = 0` representam 612,3 rpm pelo valor bruto
6123. Quando não ocorre saturação, o erro de quantização é de aproximadamente,
no máximo, metade do valor de `scale`. `scale` deve ser positivo, e todos os
valores de escala e offset devem ser finitos.

`INT16_MIN` é reservado como `ESP_TIMESERIES_INVALID_I16`. Entradas de ponto
flutuante não finitas usam esse código. Um valor finito fora do intervalo é
limitado a `INT16_MIN + 1` ou `INT16_MAX` e incrementa o contador de saturação do
canal. A API de valores brutos copia os valores sem modificá-los e conta as
ocorrências do código reservado para amostras inválidas.

O campo `encoding` não altera o comportamento do gravador. Ele permite que uma
aplicação e o software no computador concordem sobre o significado de códigos
brutos especiais, como flags de estado do sensor de corrente.

## Taxas e capacidade

Cada captura seleciona sua frequência de amostragem durante a execução. Essa
frequência deve dividir exatamente `producer_rate_hz`:

```text
sample_divider = producer_rate_hz / sample_rate_hz
```

Assim, um produtor de 1 kHz aceita 1000, 500, 250, 200, 125 ou 100 Hz, mas não
300 Hz. A decimação inteira mantém o espaçamento uniforme: a primeira chamada
do produtor é armazenada e, depois dela, uma chamada a cada `sample_divider`.

A capacidade e a duração são calculadas por:

```text
record_bytes    = channel_count * sizeof(int16_t)
sample_capacity = floor(buffer_bytes / record_bytes)
duration_s      = sample_capacity / sample_rate_hz
```

Com 128 KiB, cinco canais e 500 Hz, a capacidade é de 13.107 registros e a
captura dura aproximadamente 26,214 segundos. Os bytes restantes que não
comportam um registro completo não causam problemas.

## Máquina de estados

| Estado | Significado | Próxima operação permitida |
|---|---|---|
| `UNINITIALIZED` | A inicialização ainda não terminou | `esp_timeseries_init()` |
| `EMPTY` | Não existe captura disponível para leitura | `esp_timeseries_arm()` |
| `ARMED` | Aguardando a primeira chamada do produtor | Uma função de gravação inicia a captura |
| `CAPTURING` | Decimando e armazenando registros | Continuar as chamadas do produtor |
| `FULL` | Uma captura imutável está disponível | Ler/DUMP e chamar `esp_timeseries_clear()` |

Quando o buffer fica cheio, a aquisição é interrompida automaticamente. O
buffer nunca é circular e a captura não é sobrescrita até que o consumidor a
libere explicitamente e arme uma nova aquisição. O conteúdo é volátil e se
perde após reinicialização ou desligamento.

## Exemplo completo

```c
static const esp_timeseries_channel_t channels[] = {
    {.name = "speed", .unit = "rpm", .scale = 0.1f, .offset = 0.0f},
    {.name = "control", .unit = "%", .scale = 0.01f, .offset = 0.0f},
};

void recorder_setup(void)
{
    const esp_timeseries_config_t config = {
        .producer_rate_hz = 1000,
        .channel_count = 2,
        .channels = channels,
    };
    ESP_ERROR_CHECK(esp_timeseries_init(&config));
    ESP_ERROR_CHECK(esp_timeseries_arm(500));
}

/* Chamada por exatamente um produtor a 1 kHz. */
void control_iteration(float speed_rpm, float control_percent, int64_t now_us)
{
    const float values[] = {speed_rpm, control_percent};
    (void)esp_timeseries_record_f32(values, now_us);
}

void consume_when_full(void)
{
    esp_timeseries_capture_t capture;
    if (esp_timeseries_get_capture(&capture) == ESP_OK) {
        /* Leia ou transmita capture.samples antes de chamar CLEAR. */
        ESP_ERROR_CHECK(esp_timeseries_clear());
    }
}
```

O retorno de uma função de gravação informa se aquela chamada específica do
produtor armazenou um registro. Um retorno `false` é normal quando o gravador
está vazio, quando a decimação configurada enquanto ele está armado descarta a
chamada ou quando a captura já está cheia.

## Resumo da API

| Função | Finalidade |
|---|---|
| `esp_timeseries_init()` | Validar e publicar a configuração do singleton |
| `esp_timeseries_arm()` | Selecionar a taxa da próxima captura e reiniciar seus metadados |
| `esp_timeseries_record_f32()` | Quantizar valores físicos e armazená-los quando devido |
| `esp_timeseries_record_i16()` | Armazenar valores já codificados quando devido |
| `esp_timeseries_get_state()` | Consultar rapidamente o estado sem bloqueio |
| `esp_timeseries_get_status()` | Copiar o progresso e a geometria do buffer |
| `esp_timeseries_get_capture()` | Obter uma visualização imutável e sem cópia no estado `FULL` |
| `esp_timeseries_clear()` | Liberar a captura `FULL` e retornar ao estado `EMPTY` |
| `esp_timeseries_state_name()` | Converter um estado em uma string para protocolo ou diagnóstico |

Combine este componente com `esp_timeseries_usb_transport` quando um computador
precisar armar, consultar e baixar as capturas pela USB nativa.
