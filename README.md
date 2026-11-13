# esp_anti_windup

Coordenador ESP-IDF de controladores `esp_pid` e `esp_pr` em paralelo, com um
atuador limitado em comum. Atualiza participantes, soma pedidos não saturados,
limita a soma, distribui tracking e cancela estados parciais em caso de falha.
Sem alocação, callbacks de usuário, logging, locks ou acesso a periféricos no
caminho periódico. Licença MIT. ESP-IDF >=6.0; validado em build ESP32-S3.

## Arquitetura e contrato

Até oito participantes tipados PID/PR, registrados na inicialização. Cada um
recebe a mesma referência e medição, na mesma unidade e período. O componente
não mistura escalas ou referências individuais. Um PR pode conter seus próprios
ramos harmônicos; um grupo também pode conter várias instâncias PR/PID.

As instâncias e modelos são fornecidos e inicializados pelo consumidor.
O coordenador não escolhe ganhos, harmônicos, offsets ou parâmetros da planta.
Toda instância deve estar em modo de tracking externo, com limites locais iguais
aos do grupo. Localmente, o clamp é apenas diagnóstico: a soma usa os pedidos.
Não registre a mesma instância duas vezes nem em grupos concorrentes.

```text
requested = soma dos pedidos não saturados
applied = clamp(requested, minimum, maximum)
delta = applied - requested
alpha = 1 - exp(-dt_s / tracking_time_s)
incremento_i = peso_normalizado_i * alpha * delta
```

Tempo de tracking zero desativa a correção, mas mantém a limitação e as chamadas
que encerram o tracking pendente. Pesos são não negativos e normalizados uma
única vez na inicialização. Um PR sem ressonância ou PID com Ki=0 deve receber
peso zero; todos com peso zero permitem limitação proporcional pura. Proporcional
e derivativo continuam participando do pedido mesmo sem peso de tracking.
Um grupo PR+I com pesos 1 e 1 recebe metade da correção em cada grupo dinâmico.
O PR distribui sua parcela entre seus ramos pela API pública; não duplique-a.

A inicialização prepara alpha e pesos, valida parâmetros e reseta os
participantes. O estado operacional é pronto -> atualização -> pronto; falha
faz reset de todos. Não há estado persistido, protocolo ou thread interna.
A configuração e os ponteiros ficam em memória estática do consumidor. As
estruturas estão públicas para armazenamento, não para mutação durante operação.
O array de registro pode sair de escopo após init; instâncias/modelos não podem
mudar de endereço. Não copie/mova um proprietário que contenha as instâncias
referenciadas. Serialize setup, reset e update pelo mesmo proprietário.

## API

Inclua `esp_anti_windup.h`. Todas as unidades de saída devem coincidir; `dt_s` e
`tracking_time_s` são segundos. Referência/medição usam a unidade de entrada
comum aos participantes. Tipos e campos estão documentados no header.

- `esp_anti_windup_init(group, config)`: task context, antes da aquisição.
  Recebe limites finitos ordenados, dt positivo, tracking não negativo e lista
  de 1..8 participantes válidos. Rejeita duplicatas e tracking pendente. O modelo
  PR não retém sample rate: o consumidor deve prepará-lo para o dt do grupo.
  PID com máximo dt habilitado deve aceitar estritamente o período escolhido.
  Copia registro e prepara pesos. Argumentos inválidos deixam grupo e estados
  intactos; se reset de participante falhar, anteriores podem ter sido resetados.
- `esp_anti_windup_update(group, reference, measurement, result)`: atualiza todos,
  soma/clampa e aplica tracking uma vez. Retorna `ESP_OK` e snapshot com pedidos
  em ordem de registro, soma, comando aplicado, delta e flag de saturação.
  Erro numérico/de controlador reseta todos e zera resultado válido; nunca envie
  ao atuador um resultado de chamada malsucedida. Preserva o erro original;
  falha do reset pode ser inspecionada por uma chamada explícita de reset.
- `esp_anti_windup_reset(group)`: zera participantes e cancela tracking pendente,
  preservando configuração. Tenta todos e retorna o primeiro erro.

Ponteiros/argumentos inválidos retornam `ESP_ERR_INVALID_ARG`; grupo não
inicializado retorna `ESP_ERR_INVALID_STATE` nas funções operacionais. As APIs
PID/PR podem propagar seus erros. Resultado nulo não avança estados. Um erro
com destino válido zera o resultado. Não faça update/apply_tracking diretamente
nos participantes enquanto o coordenador for proprietário.

## Exemplo e instalação

[examples/pr_pid.c](examples/pr_pid.c) contém `app_main` completo com PR a 60 Hz
mais integral, pesos iguais e período de 14,4 kHz. É exercício numérico, sem
planta, ADC ou DAC, e seus ganhos não são recomendação de sintonia.

Para experimentá-lo, copie o arquivo para `main/main.c` de um projeto ESP-IDF:

```cmake
idf_component_register(SRCS "main.c" INCLUDE_DIRS "." REQUIRES esp_anti_windup)
```

Depois de publicar este repositório, **o consumidor** lista apenas o coordenador
em `main/idf_component.yml`:

```yaml
dependencies:
  esp_anti_windup:
    git: https://github.com/smartsensingme/esp_anti_windup.git
    version: <hash-completo-publicado-e-testado>
```

Substitua o placeholder antes do uso. O manifesto **na raiz deste componente**
resolve transitivamente PID `6f0e509` e PR `107f4ce`, ambos fixados por hash
completo e URL HTTPS. O CMake declara `REQUIRES esp_pid esp_pr` para compilar e
exportar os headers. Não há cópias internas dos controladores.

Instalação manual/offline: coloque os três repositórios em `components/`.
Para evitar obtenção pela rede, use overrides locais explícitos nos manifestos
ou `IDF_COMPONENT_MANAGER=0` quando TODAS as dependências já estiverem disponíveis.
Não presuma precedência implícita entre cópias locais e gerenciadas.

```sh
idf.py reconfigure build
```

## Memória, otimização e temporização

`CONFIG_ESP_ANTI_WINDUP_IRAM` (padrão y) coloca update/reset em IRAM. Init usa
libm e fica em flash. O loop faz dispatch direto tipado, sem callbacks de
usuário, divisão ou exponencial por amostra. O trabalho é limitado por oito
participantes mais o custo dos respectivos controladores. RAM depende das
estruturas fixas e da stack dos controladores. Menos participantes reduzem o
trabalho executado, mas o armazenamento tem capacidade fixa de oito.

IRAM consome memória interna e não garante maior velocidade ou execução completa
sem cache. Confira ELF, helpers do compilador, instâncias, modelos, stack e
chamadores. O coordenador não escreve flash nem controla afinidade, aquisição,
escalonamento, watchdogs ou deadlines. Meça o caminho completo no alvo antes de
afirmar ganho de desempenho. Não use fast-math sem revalidar NaN/Inf e estabilidade.

## Testes host

Compilam os componentes reais com stubs ESP-IDF fornecidos pelos testes PID:

```sh
cmake -S tests -B build/tests \
  -DESP_PID_SOURCE_DIR=/caminho/esp_pid \
  -DESP_PR_SOURCE_DIR=/caminho/esp_pr
cmake --build build/tests
ctest --test-dir build/tests --output-on-failure
```

Cobrem pesos desiguais, saturação apenas da soma, pedidos opostos, registro
inválido/duplicado, falha parcial com reset, proporcional puro, tracking inativo
e recuperação após reversão. A aplicação consumidora também verifica PR+I,
resposta DC/AC ilustrativa e telemetria. Testes numéricos não identificam planta.

## Diagnóstico e migração

Saturação prolongada exige investigar ganho, polaridade, offset e autoridade do
atuador. A política de parar por saturação persistente pertence à aplicação.
O comando não inclui quantização, slew rate ou outras limitações físicas; não
é medição de corrente. Retorne ao neutro apropriado ao hardware após erro.

Esta versão substitui o anterior núcleo portátil `anti_windup` e absorve a
coordenação antes feita em `app_parallel`. A API anterior init/compute foi
removida; não há compatibilidade de fonte ou ABI com ela. Apenas ESP-IDF está
no escopo. O histórico Git do núcleo anterior foi preservado.
