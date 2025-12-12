---
description: Os operadores RxJS são categorizados em sete grupos - transformação, filtragem, combinação, utilitários, condicionais, tratamento de erros e multicasting. Aprenda o uso prático em TypeScript com listas abrangentes de operadores e conceitos de pipeline.
---

# Compreendendo os Operadores

Os operadores RxJS são um conjunto de funções para transformar, compor e controlar fluxos de dados Observable.

Os operadores geralmente são usados em combinação com vários outros, e é aí que entra o "pipeline".
- [O que é o Pipeline RxJS](./pipeline.md)

No RxJS, os operadores se dividem nas seguintes categorias


## Lista de categorias

- [Operadores de Transformação](./transformation/)
- [Operadores de Filtragem](./filtering/)
- [Operadores de Combinação](./combination/)
- [Operadores Utilitários](./utility/)
- [Operadores Condicionais](./conditional/)
- [Operadores de Tratamento de Erros](../error-handling/strategies.md)
- [Operadores de Multicasting](./multicasting/)

Cada categoria contém vários operadores úteis.
Consulte cada categoria para obter detalhes.


## Lista de Operadores

Para uma descrição detalhada de cada operador, clique no link para navegar.

<table style="overflow: visible;">
  <caption>
   Lista de categorias de Operadores
  </caption>
  <thead>
    <tr>
      <th scope="col">Categoria</th>
      <th scope="col">Operador</th>
      <th scope="col">Descrição</th>
    </tr>
  </thead>
  <tbody>
    <!-- Operadores de Transformação -->
    <tr>
      <th scope="row" rowspan="15"><a href="./transformation/">Transformação</a></th>
      <td><a href="./transformation/map.html">map</a></td>
      <td>Converte cada valor</td>
    </tr>
    <tr>
      <td><a href="./transformation/scan.html">scan</a></td>
      <td>Acumula valores e gera resultados intermediários</td>
    </tr>
    <tr>
      <td><a href="./transformation/reduce.html">reduce</a></td>
      <td>Acumula todos os valores e gera apenas o resultado final</td>
    </tr>
    <tr>
      <td><a href="./transformation/pairwise.html">pairwise</a></td>
      <td>Processa dois valores consecutivos em pares</td>
    </tr>
    <tr>
      <td><a href="./transformation/groupBy.html">groupBy</a></td>
      <td>Agrupa streams por chave</td>
    </tr>
    <tr>
      <td><a href="./transformation/mergeMap.html">mergeMap</a></td>
      <td>Execução paralela de processamento assíncrono</td>
    </tr>
    <tr>
      <td><a href="./transformation/switchMap.html">switchMap</a></td>
      <td>Executa apenas o processamento assíncrono mais recente (cancela processamento mais antigo)</td>
    </tr>
    <tr>
      <td><a href="./transformation/concatMap.html">concatMap</a></td>
      <td>Executa processos assíncronos sequencialmente</td>
    </tr>
    <tr>
      <td><a href="./transformation/exhaustMap.html">exhaustMap</a></td>
      <td>Ignora novos processos durante a execução</td>
    </tr>
    <tr>
      <td><a href="./transformation/expand.html">expand</a></td>
      <td>Expande resultados recursivamente</td>
    </tr>
    <tr>
      <td><a href="./transformation/buffer.html">buffer</a></td>
      <td>Publica valores em um array</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferTime.html">bufferTime</a></td>
      <td>Publica valores em intervalos de tempo especificados</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferCount.html">bufferCount</a></td>
      <td>Publica valores em lotes de número especificado de valores</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferWhen.html">bufferWhen</a></td>
      <td>Buffering com condições de término controladas dinamicamente</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferToggle.html">bufferToggle</a></td>
      <td>Buffering com controle independente de início e fim</td>
    </tr>
    <!-- Operadores de Filtragem -->
    <tr>
      <th scope="row" rowspan="22"><a href="./filtering/">Filtragem</a></th>
      <td><a href="./filtering/filter.html">filter</a></td>
      <td>Permite passar apenas valores que correspondem à condição</td>
    </tr>
    <tr>
      <td><a href="./filtering/take.html">take</a></td>
      <td>Obtém apenas os primeiros N valores</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeLast.html">takeLast</a></td>
      <td>Obtém os últimos N valores</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeWhile.html">takeWhile</a></td>
      <td>Obtém valores enquanto a condição é atendida</td>
    </tr>
    <tr>
      <td><a href="./filtering/skip.html">skip</a></td>
      <td>Pula os primeiros N valores</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipLast.html">skipLast</a></td>
      <td>Pula os últimos N valores</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipWhile.html">skipWhile</a></td>
      <td>Pula valores enquanto a condição é satisfeita</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipUntil.html">skipUntil</a></td>
      <td>Pula valores até que outro Observable dispare</td>
    </tr>
    <tr>
      <td><a href="./filtering/first.html">first</a></td>
      <td>Obtém o primeiro valor ou o primeiro valor que satisfaz uma condição</td>
    </tr>
    <tr>
      <td><a href="./filtering/last.html">last</a></td>
      <td>Obtém o último valor ou o último valor que satisfaz a condição</td>
    </tr>
    <tr>
      <td><a href="./filtering/elementAt.html">elementAt</a></td>
      <td>Obtém o valor em um determinado índice</td>
    </tr>
    <tr>
      <td><a href="./filtering/find.html">find</a></td>
      <td>Encontra o primeiro valor que satisfaz uma condição</td>
    </tr>
    <tr>
      <td><a href="./filtering/findIndex.html">findIndex</a></td>
      <td>Obtém o índice do primeiro valor que satisfaz a condição</td>
    </tr>
    <tr>
      <td><a href="./filtering/debounceTime.html">debounceTime</a></td>
      <td>Emite o último valor se nenhuma entrada for recebida por um tempo especificado</td>
    </tr>
    <tr>
      <td><a href="./filtering/throttleTime.html">throttleTime</a></td>
      <td>Passa o primeiro valor e ignora o novo valor pelo tempo especificado</td>
    </tr>
    <tr>
      <td><a href="./filtering/auditTime.html">auditTime</a></td>
      <td>Emite o último valor após o tempo especificado</td>
    </tr>
    <tr>
      <td><a href="./filtering/audit.html">audit</a></td>
      <td>Emite o último valor com Observable personalizado para controlar o período</td>
    </tr>
    <tr>
      <td><a href="./filtering/sampleTime.html">sampleTime</a></td>
      <td>Amostra o valor mais recente em intervalo de tempo especificado</td>
    </tr>
    <tr>
      <td><a href="./filtering/ignoreElements.html">ignoreElements</a></td>
      <td>Ignora todos os valores e passa apenas conclusões/erros</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinct.html">distinct</a></td>
      <td>Remove todos os valores duplicados (gera apenas valores únicos)</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilChanged.html">distinctUntilChanged</a></td>
      <td>Remove valores duplicados consecutivos</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilKeyChanged.html">distinctUntilKeyChanged</a></td>
      <td>Detecta apenas mudanças em propriedades específicas de um objeto</td>
    </tr>
    <!-- Operadores de Combinação (Pipeable) -->
    <tr>
      <th scope="row" rowspan="12"><a href="./combination/">Combinação (Pipeable)</a></th>
      <td><a href="./combination/concatWith.html">concatWith</a></td>
      <td>Junta outros Observables em sequência após a conclusão</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeWith.html">mergeWith</a></td>
      <td>Combina vários Observables simultaneamente</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestWith.html">combineLatestWith</a></td>
      <td>Combina o valor mais recente de cada Observable</td>
    </tr>
    <tr>
      <td><a href="./combination/zipWith.html">zipWith</a></td>
      <td>Emparelha valores na ordem correspondente</td>
    </tr>
    <tr>
      <td><a href="./combination/raceWith.html">raceWith</a></td>
      <td>Adota apenas o primeiro Observable que dispara</td>
    </tr>
    <tr>
      <td><a href="./combination/withLatestFrom.html">withLatestFrom</a></td>
      <td>Anexa outros valores mais recentes ao stream principal</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeAll.html">mergeAll</a></td>
      <td>Achata Higher-order Observables em paralelo</td>
    </tr>
    <tr>
      <td><a href="./combination/concatAll.html">concatAll</a></td>
      <td>Achata Higher-order Observable em sequência</td>
    </tr>
    <tr>
      <td><a href="./combination/switchAll.html">switchAll</a></td>
      <td>Muda para o Higher-order Observable mais recente</td>
    </tr>
    <tr>
      <td><a href="./combination/exhaustAll.html">exhaustAll</a></td>
      <td>Ignora novo Higher-order Observable durante a execução</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestAll.html">combineLatestAll</a></td>
      <td>Combina os valores mais recentes de todos os Observables internos</td>
    </tr>
    <tr>
      <td><a href="./combination/zipAll.html">zipAll</a></td>
      <td>Emparelha os valores correspondentes de cada Observable interno</td>
    </tr>
    <!-- Operadores Utilitários -->
    <tr>
      <th scope="row" rowspan="15"><a href="./utility/">Utilitários</a></th>
      <td><a href="./utility/tap.html">tap</a></td>
      <td>Executa efeitos colaterais (ex: saída de log)</td>
    </tr>
    <tr>
      <td><a href="./utility/finalize.html">finalize</a></td>
      <td>Executa pós-processamento na conclusão ou erro</td>
    </tr>
    <tr>
      <td><a href="./utility/delay.html">delay</a></td>
      <td>Atrasa todos os valores por um tempo especificado</td>
    </tr>
    <tr>
      <td><a href="./utility/delayWhen.html">delayWhen</a></td>
      <td>Atrasa cada valor dinamicamente com um Observable separado</td>
    </tr>
    <tr>
      <td><a href="./utility/timeout.html">timeout</a></td>
      <td>Emite um erro se um valor não chegar dentro de um tempo especificado</td>
    </tr>
    <tr>
      <td><a href="./utility/takeUntil.html">takeUntil</a></td>
      <td>Recupera valores até que outro Observable emita um valor</td>
    </tr>
    <tr>
      <td><a href="./utility/retry.html">retry</a></td>
      <td>Tenta novamente até o número especificado de vezes em caso de erro</td>
    </tr>
    <tr>
      <td><a href="./utility/repeat.html">repeat</a></td>
      <td>Repete um número especificado de vezes após a conclusão</td>
    </tr>
    <tr>
      <td><a href="./utility/startWith.html">startWith</a></td>
      <td>Adiciona um valor inicial ao início do stream</td>
    </tr>
    <tr>
      <td><a href="./utility/toArray.html">toArray</a></td>
      <td>Publica todos os valores juntos em um array</td>
    </tr>
    <tr>
      <td><a href="./utility/materialize.html">materialize</a></td>
      <td>Converte uma notificação em um objeto Notification</td>
    </tr>
    <tr>
      <td><a href="./utility/dematerialize.html">dematerialize</a></td>
      <td>Converte o objeto Notification de volta para uma notificação normal</td>
    </tr>
    <tr>
      <td><a href="./utility/observeOn.html">observeOn</a></td>
      <td>Usa o scheduler para controlar quando os valores são publicados</td>
    </tr>
    <tr>
      <td><a href="./utility/subscribeOn.html">subscribeOn</a></td>
      <td>Usa o scheduler para controlar quando iniciar a inscrição</td>
    </tr>
    <tr>
      <td><a href="./utility/timestamp.html">timestamp</a></td>
      <td>Adiciona um timestamp a cada valor</td>
    </tr>
    <!-- Operadores Condicionais -->
    <tr>
      <th scope="row" rowspan="3"><a href="./conditional/">Condicional</a></th>
      <td><a href="./conditional/defaultIfEmpty.html">defaultIfEmpty</a></td>
      <td>Se nenhum valor estiver disponível, emite valor padrão</td>
    </tr>
    <tr>
      <td><a href="./conditional/every.html">every</a></td>
      <td>Determina se todos os valores satisfazem a condição</td>
    </tr>
    <tr>
      <td><a href="./conditional/isEmpty.html">isEmpty</a></td>
      <td>Determina se nenhum valor foi emitido</td>
    </tr>
    <!-- Tratamento de Erros -->
    <tr>
      <th scope="row" rowspan="3"><a href="../error-handling/strategies.html">Tratamento de Erros</a></th>
      <td><a href="../error-handling/retry-catch.html">catchError</a></td>
      <td>Captura erros e executa processamento de fallback</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retry</a></td>
      <td>Tenta novamente um número especificado de vezes em caso de erro</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retryWhen</a></td>
      <td>Tenta novamente com condições personalizadas</td>
    </tr>
    <!-- Multicasting -->
    <tr>
      <th scope="row" rowspan="2"><a href="./multicasting/">Multicasting</a></th>
      <td><a href="./multicasting/share.html">share</a></td>
      <td>Compartilha Observable entre vários assinantes</td>
    </tr>
    <tr>
      <td><a href="./multicasting/shareReplay.html">shareReplay</a></td>
      <td>Armazena em cache os últimos N valores e os reproduz para novos assinantes</td>
    </tr>
  </tbody>
</table>
