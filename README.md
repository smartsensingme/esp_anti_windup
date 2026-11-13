# anti_windup

Componente independente em C99 para calcular o anti-windup de controladores
em paralelo que compartilham um atuador limitado. Não depende de RTOS, ESP-IDF,
PID ou PR. Licença MIT. A integração ESP-IDF foi validada com v6.0 / ESP32-S3;
a integração Zephyr abaixo é um caminho CMake genérico, ainda sem teste em alvo.

## Responsabilidades e algoritmo

Some os pedidos **não saturados** dos controladores na mesma unidade de saída.
Limite a soma uma única vez e distribua o incremento de tracking entre os
estados dinâmicos. Não some as saídas já limitadas de cada controlador.

```text
requested  = soma dos pedidos
applied    = clamp(requested, minimum, maximum)
error      = applied - requested
alpha      = 1 - exp(-dt / tracking_time)
correction = alpha * error
```

A exponencial é calculada apenas na inicialização. Tempo de tracking zero
**desativa a correção**, mantendo o limitador; não significa correção instantânea.
A relaxação isolada tem alpha entre zero e um; isso não garante estabilidade
da planta nem escolhe os ganhos dos controladores.

O componente não possui acumulador, callbacks ou estado de PID/PR. Não realiza
I/O, persistência, logging, alocação ou detecção de saturação prolongada. O
consumidor decide a distribuição da correção e o tratamento de falhas. Não há
wire protocol, formato persistido ou dependências externas transitivas.

## API, unidades e ciclo de vida

Inclua `anti_windup.h`. `anti_windup_init()` recebe configuração de destino,
limite inferior e superior, período em segundos e tempo de tracking em segundos.
Os limites devem ser finitos e estritamente ordenados; o período deve ser finito
e positivo; o tempo de tracking, finito e não negativo. Prepare antes do loop.

`anti_windup_t` guarda `minimum`, `maximum` e `alpha`. A memória pertence ao
chamador, deve permanecer válida e ser tratada como imutável após inicializar.
Não há destruição ou reset: para mudar limites ou período, prepare outra
configuração fora do caminho crítico e troque-a com sincronização apropriada.

`anti_windup_compute()` recebe configuração, pedido total e destino do resultado.
`anti_windup_result_t` contém `requested`, `applied`, `error`, `correction` e
`saturated`. Todos os campos numéricos usam a unidade do comando do atuador;
`correction` já é um incremento completo, não uma derivada a multiplicar por dt.
`saturated` indica que houve limitação nesta amostra, sem tolerância ou histerese.

As funções são públicas e não chamam uma à outra. Retornam `ANTI_WINDUP_OK` ou
`ANTI_WINDUP_INVALID_ARGUMENT`. Ponteiros nulos, valores inválidos e overflow
na diferença/correção são rejeitados sem modificar o destino. A aplicação deve
verificar o retorno e escolher seu comando de falha; uma falha não zera a saída
automaticamente. Não leia resultado sem sucesso.

Chamadas de cálculo podem compartilhar configuração imutável e usar resultados
separados. Não há locks; não altere configuração enquanto outra thread a usa.
O cálculo tem trabalho constante e nenhuma função transcendental; não há uma
promessa de número fixo de ciclos. Prefira executar em contexto de tarefa.

## Exemplo mínimo

```c
#include "anti_windup.h"

anti_windup_t aw;
/* Fora do loop: saída em volts, amostragem de 1 ms, tracking de 20 ms. */
if (anti_windup_init(&aw, -3.0f, 3.0f, 0.001f, 0.020f) != ANTI_WINDUP_OK) {
    /* Tratar erro e não iniciar o loop. */
}
/* Dentro do loop, depois de atualizar os dois controladores: */
anti_windup_result_t result;
if (anti_windup_compute(&aw, requested_a + requested_b, &result) == ANTI_WINDUP_OK) {
    /* Enviar result.applied ao atuador e aplicar uma vez os incrementos: */
    /* tracking_a(0.5f * result.correction); */
    /* tracking_b(0.5f * result.correction); */
}
```

O trecho mostra a sequência; `requested_a/b` são saídas da aplicação.
[examples/parallel_integrators.c](examples/parallel_integrators.c) é executável
completo com dois integradores, saturação e inversão do erro. Não é modelo de
planta nem receita de sintonia.

### PID e PR em paralelo

Selecione tracking **externo** nos dois componentes. Atualize cada um com sua
API `update_result()`, some os campos `requested_output`, chame
`anti_windup_compute()` e depois as APIs `apply_tracking()` de cada componente.
Os pesos não negativos devem somar um entre os grupos dinâmicos participantes:
por exemplo, metade para o integral e metade para o conjunto PR. Um único grupo
dinâmico recebe tudo; proporcional puro não recebe correção de estado.

O PR distribui internamente a parcela recebida entre seus ramos. Não duplique a
correção em cada ramo. Não aplique novamente alpha ou dt. Complete tracking mesmo
com correção zero se a API do controlador exigir. Se uma etapa falhar, não envie
um comando parcial: a aplicação precisa resolver/resetar os estados pendentes.
O limitador trabalha antes da quantização; tracking de quantização, slew rate ou
outros limites físicos não está incluído nesta API.

## Build portátil, testes e exemplo

```sh
cmake -S . -B build -DANTI_WINDUP_BUILD_TESTS=ON -DANTI_WINDUP_BUILD_EXAMPLES=ON
cmake --build build
ctest --test-dir build --output-on-failure
./build/parallel_integrators
```

O target `anti_windup` exporta o diretório `include` e requer C99; libm é
vinculada em toolchains não MSVC. Para integração manual, compile
`anti_windup.c`, acrescente `include` e vincule a biblioteca matemática quando
necessário. Sem Kconfig ou cabeçalhos de plataforma nesse modo.

Os testes cobrem limites assimétricos, soma de pedidos opostos, correção
inativa, entradas inválidas/overflow sem alteração do destino e recuperação
após saturação. O exemplo também é compilado no fluxo acima.

## ESP-IDF

O repositório funciona diretamente como componente na raiz. Instale em
`components/anti_windup` (clone ou submódulo); adicione `anti_windup` a `REQUIRES`
do consumidor. Não há dependências externas a resolver.

Quando este repositório tiver sido publicado em
`https://github.com/smartsensingme/anti_windup.git`, o **consumidor** poderá usar
em seu `main/idf_component.yml`:

```yaml
dependencies:
  anti_windup:
    git: https://github.com/smartsensingme/anti_windup.git
    version: <hash-completo-do-commit-publicado-e-testado>
```

O placeholder deve ser substituído antes do uso. O `idf_component.yml` na raiz
deste componente declara apenas a versão suportada da ESP-IDF, sem dependências
transitivas. Para instalação offline, use o diretório local e
`IDF_COMPONENT_MANAGER=0` se o projeto inteiro puder prescindir do gerenciador;
projetos com outras dependências gerenciadas devem mantê-lo habilitado e usar
um `override_path` explícito no manifesto consumidor. Não dependa de precedência
implícita entre componentes locais e gerenciados.

`CONFIG_ANTI_WINDUP_IRAM` (padrão y) coloca `anti_windup_compute()` em IRAM;
a inicialização permanece em flash. O custo é RAM interna de instruções. O
atributo não garante maior velocidade ou execução completa com cache desligado:
confira ELF, chamadores, dados, stack, helpers do compilador e disponibilidade
do escalonador. O componente não escreve flash.

## Zephyr e outros alvos

Após o `find_package(Zephyr ...)` e a criação do target da aplicação:

```cmake
add_subdirectory(path/to/anti_windup anti_windup-build)
target_link_libraries(app PRIVATE anti_windup)
```

O modo portátil usa a biblioteca matemática disponível no toolchain. Se a
plataforma exigir localização especial, defina `ANTI_WINDUP_FAST_ATTR` ao
compilar o fonte (por exemplo, um atributo de seção); o padrão portátil é vazio.
Essa macro não adiciona headers nem tipos de RTOS à API. Revalide o link e o mapa
no alvo; integração Zephyr e temporização em hardware ainda não foram medidas.

## Diagnóstico e compatibilidade

Correção zero pode significar ausência de saturação ou tempo de tracking zero.
Saturação prolongada pode indicar ganho, polaridade, offset ou autoridade do
atuador inadequados; este componente não identifica a causa. Oscilação após
habilitar tracking exige rever tempo de tracking, pesos e dinâmica da planta.
Não use `-ffast-math` sem revalidar: a rejeição de NaN/Inf depende de semântica
IEEE de ponto flutuante. Configurações e resultados não são formatos de arquivo
ou ABI persistente. Primeira versão pública: 0.1.0.
