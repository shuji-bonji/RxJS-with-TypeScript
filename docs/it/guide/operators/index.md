---
description: "Gli operatori RxJS classificati per categoria, con una panoramica completa delle funzionalità, degli usi e degli operatori rappresentativi. Organizzati in 7 categorie (trasformazione, filtraggio, combinazione, utility, condizionale, gestione errori, multicasting) con il concetto di pipeline per apprendere l'uso pratico con TypeScript."
---

# Comprensione degli Operatori

Gli operatori RxJS sono un insieme di funzioni per trasformare, comporre e controllare i flussi di dati Observable.

Gli operatori vengono solitamente utilizzati in combinazione, ed è qui che entra in gioco la "pipeline".
- [Cos'è la pipeline di RxJS](./pipeline.md)

In RxJS, gli operatori sono classificati nelle seguenti categorie.


## Elenco delle Categorie

- [Operatori di Trasformazione](./transformation/)
- [Operatori di Filtraggio](./filtering/)
- [Operatori di Combinazione](./combination/)
- [Operatori di Utility](./utility/)
- [Operatori Condizionali](./conditional/)
- [Operatori di Gestione Errori](../error-handling/strategies.md)
- [Operatori di Multicasting](./multicasting/)

Ogni categoria contiene numerosi operatori utili.
Per i dettagli, consultare ciascuna categoria.


## Tabella Riepilogativa degli Operatori

Per una spiegazione dettagliata di ciascun operatore, cliccare sul link corrispondente.

<table style="overflow: visible;">
  <caption>
   Elenco delle categorie di Operator
  </caption>
  <thead>
    <tr>
      <th scope="col">Categoria</th>
      <th scope="col">Operator</th>
      <th scope="col">Descrizione</th>
    </tr>
  </thead>
  <tbody>
    <!-- Operatori di Trasformazione -->
    <tr>
      <th scope="row" rowspan="15"><a href="./transformation/">Trasformazione</a></th>
      <td><a href="./transformation/map.html">map</a></td>
      <td>Trasforma ogni valore</td>
    </tr>
    <tr>
      <td><a href="./transformation/scan.html">scan</a></td>
      <td>Accumula valori ed emette anche risultati intermedi</td>
    </tr>
    <tr>
      <td><a href="./transformation/reduce.html">reduce</a></td>
      <td>Accumula tutti i valori ed emette solo il risultato finale</td>
    </tr>
    <tr>
      <td><a href="./transformation/pairwise.html">pairwise</a></td>
      <td>Elabora due valori consecutivi come coppia</td>
    </tr>
    <tr>
      <td><a href="./transformation/groupBy.html">groupBy</a></td>
      <td>Raggruppa i flussi per chiave</td>
    </tr>
    <tr>
      <td><a href="./transformation/mergeMap.html">mergeMap</a></td>
      <td>Esegue elaborazioni asincrone in parallelo</td>
    </tr>
    <tr>
      <td><a href="./transformation/switchMap.html">switchMap</a></td>
      <td>Esegue solo l'elaborazione asincrona più recente (annulla quelle precedenti)</td>
    </tr>
    <tr>
      <td><a href="./transformation/concatMap.html">concatMap</a></td>
      <td>Esegue elaborazioni asincrone in sequenza</td>
    </tr>
    <tr>
      <td><a href="./transformation/exhaustMap.html">exhaustMap</a></td>
      <td>Ignora nuove elaborazioni mentre una è in corso</td>
    </tr>
    <tr>
      <td><a href="./transformation/expand.html">expand</a></td>
      <td>Espande i risultati ricorsivamente</td>
    </tr>
    <tr>
      <td><a href="./transformation/buffer.html">buffer</a></td>
      <td>Raggruppa i valori in un array ed emette</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferTime.html">bufferTime</a></td>
      <td>Raggruppa ed emette i valori a intervalli di tempo specificati</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferCount.html">bufferCount</a></td>
      <td>Raggruppa ed emette i valori per numero specificato</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferWhen.html">bufferWhen</a></td>
      <td>Buffering con controllo dinamico della condizione di fine</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferToggle.html">bufferToggle</a></td>
      <td>Buffering con controllo indipendente di inizio e fine</td>
    </tr>
    <!-- Operatori di Filtraggio -->
    <tr>
      <th scope="row" rowspan="22"><a href="./filtering/">Filtraggio</a></th>
      <td><a href="./filtering/filter.html">filter</a></td>
      <td>Lascia passare solo i valori che soddisfano la condizione</td>
    </tr>
    <tr>
      <td><a href="./filtering/take.html">take</a></td>
      <td>Ottiene solo i primi N valori</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeLast.html">takeLast</a></td>
      <td>Ottiene gli ultimi N valori</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeWhile.html">takeWhile</a></td>
      <td>Ottiene i valori finché la condizione è soddisfatta</td>
    </tr>
    <tr>
      <td><a href="./filtering/skip.html">skip</a></td>
      <td>Salta i primi N valori</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipLast.html">skipLast</a></td>
      <td>Salta gli ultimi N valori</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipWhile.html">skipWhile</a></td>
      <td>Salta i valori finché la condizione è soddisfatta</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipUntil.html">skipUntil</a></td>
      <td>Salta i valori fino a quando un altro Observable emette</td>
    </tr>
    <tr>
      <td><a href="./filtering/first.html">first</a></td>
      <td>Ottiene il primo valore o il primo valore che soddisfa la condizione</td>
    </tr>
    <tr>
      <td><a href="./filtering/last.html">last</a></td>
      <td>Ottiene l'ultimo valore o l'ultimo valore che soddisfa la condizione</td>
    </tr>
    <tr>
      <td><a href="./filtering/elementAt.html">elementAt</a></td>
      <td>Ottiene il valore all'indice specificato</td>
    </tr>
    <tr>
      <td><a href="./filtering/find.html">find</a></td>
      <td>Trova il primo valore che soddisfa la condizione</td>
    </tr>
    <tr>
      <td><a href="./filtering/findIndex.html">findIndex</a></td>
      <td>Ottiene l'indice del primo valore che soddisfa la condizione</td>
    </tr>
    <tr>
      <td><a href="./filtering/debounceTime.html">debounceTime</a></td>
      <td>Emette l'ultimo valore se non ci sono input per il tempo specificato</td>
    </tr>
    <tr>
      <td><a href="./filtering/throttleTime.html">throttleTime</a></td>
      <td>Lascia passare il primo valore e ignora i nuovi valori per il tempo specificato</td>
    </tr>
    <tr>
      <td><a href="./filtering/auditTime.html">auditTime</a></td>
      <td>Emette l'ultimo valore dopo il tempo specificato</td>
    </tr>
    <tr>
      <td><a href="./filtering/audit.html">audit</a></td>
      <td>Emette l'ultimo valore controllando il periodo con Observable personalizzato</td>
    </tr>
    <tr>
      <td><a href="./filtering/sampleTime.html">sampleTime</a></td>
      <td>Campiona il valore più recente a intervalli di tempo specificati</td>
    </tr>
    <tr>
      <td><a href="./filtering/ignoreElements.html">ignoreElements</a></td>
      <td>Ignora tutti i valori e lascia passare solo completamento/errore</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinct.html">distinct</a></td>
      <td>Rimuove tutti i valori duplicati (emette solo valori unici)</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilChanged.html">distinctUntilChanged</a></td>
      <td>Rimuove i valori duplicati consecutivi</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilKeyChanged.html">distinctUntilKeyChanged</a></td>
      <td>Rileva solo le modifiche di una proprietà specifica dell'oggetto</td>
    </tr>
    <!-- Operatori di Combinazione (Pipeable) -->
    <tr>
      <th scope="row" rowspan="12"><a href="./combination/">Combinazione (Pipeable)</a></th>
      <td><a href="./combination/concatWith.html">concatWith</a></td>
      <td>Combina altri Observable in sequenza dopo il completamento</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeWith.html">mergeWith</a></td>
      <td>Combina più Observable simultaneamente</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestWith.html">combineLatestWith</a></td>
      <td>Combina i valori più recenti di ciascun Observable</td>
    </tr>
    <tr>
      <td><a href="./combination/zipWith.html">zipWith</a></td>
      <td>Accoppia i valori in ordine corrispondente</td>
    </tr>
    <tr>
      <td><a href="./combination/raceWith.html">raceWith</a></td>
      <td>Adotta solo l'Observable che emette per primo</td>
    </tr>
    <tr>
      <td><a href="./combination/withLatestFrom.html">withLatestFrom</a></td>
      <td>Aggiunge gli ultimi valori di altri al flusso principale</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeAll.html">mergeAll</a></td>
      <td>Appiattisce Higher-order Observable in parallelo</td>
    </tr>
    <tr>
      <td><a href="./combination/concatAll.html">concatAll</a></td>
      <td>Appiattisce Higher-order Observable in sequenza</td>
    </tr>
    <tr>
      <td><a href="./combination/switchAll.html">switchAll</a></td>
      <td>Passa all'Higher-order Observable più recente</td>
    </tr>
    <tr>
      <td><a href="./combination/exhaustAll.html">exhaustAll</a></td>
      <td>Ignora nuovi Higher-order Observable mentre uno è in corso</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestAll.html">combineLatestAll</a></td>
      <td>Combina i valori più recenti di tutti gli Observable interni</td>
    </tr>
    <tr>
      <td><a href="./combination/zipAll.html">zipAll</a></td>
      <td>Accoppia i valori corrispondenti di ciascun Observable interno</td>
    </tr>
    <!-- Operatori di Utility -->
    <tr>
      <th scope="row" rowspan="15"><a href="./utility/">Utility</a></th>
      <td><a href="./utility/tap.html">tap</a></td>
      <td>Esegue effetti collaterali (come output di log)</td>
    </tr>
    <tr>
      <td><a href="./utility/finalize.html">finalize</a></td>
      <td>Esegue post-elaborazione al completamento o in caso di errore</td>
    </tr>
    <tr>
      <td><a href="./utility/delay.html">delay</a></td>
      <td>Ritarda tutti i valori per il tempo specificato</td>
    </tr>
    <tr>
      <td><a href="./utility/delayWhen.html">delayWhen</a></td>
      <td>Ritarda dinamicamente ciascun valore con un altro Observable</td>
    </tr>
    <tr>
      <td><a href="./utility/timeout.html">timeout</a></td>
      <td>Emette un errore se non arrivano valori entro il tempo specificato</td>
    </tr>
    <tr>
      <td><a href="./utility/takeUntil.html">takeUntil</a></td>
      <td>Ottiene i valori fino a quando un altro Observable emette un valore</td>
    </tr>
    <tr>
      <td><a href="./utility/retry.html">retry</a></td>
      <td>Riprova fino al numero specificato di volte in caso di errore</td>
    </tr>
    <tr>
      <td><a href="./utility/repeat.html">repeat</a></td>
      <td>Ripete il numero specificato di volte dopo il completamento</td>
    </tr>
    <tr>
      <td><a href="./utility/startWith.html">startWith</a></td>
      <td>Aggiunge un valore iniziale all'inizio del flusso</td>
    </tr>
    <tr>
      <td><a href="./utility/toArray.html">toArray</a></td>
      <td>Raggruppa tutti i valori in un array ed emette</td>
    </tr>
    <tr>
      <td><a href="./utility/materialize.html">materialize</a></td>
      <td>Converte le notifiche in oggetti Notification</td>
    </tr>
    <tr>
      <td><a href="./utility/dematerialize.html">dematerialize</a></td>
      <td>Riconverte gli oggetti Notification in notifiche normali</td>
    </tr>
    <tr>
      <td><a href="./utility/observeOn.html">observeOn</a></td>
      <td>Controlla il timing di emissione dei valori con uno scheduler</td>
    </tr>
    <tr>
      <td><a href="./utility/subscribeOn.html">subscribeOn</a></td>
      <td>Controlla il timing di inizio sottoscrizione con uno scheduler</td>
    </tr>
    <tr>
      <td><a href="./utility/timestamp.html">timestamp</a></td>
      <td>Aggiunge un timestamp a ciascun valore</td>
    </tr>
    <!-- Operatori Condizionali -->
    <tr>
      <th scope="row" rowspan="3"><a href="./conditional/">Condizionale</a></th>
      <td><a href="./conditional/defaultIfEmpty.html">defaultIfEmpty</a></td>
      <td>Emette un valore predefinito se non ci sono valori</td>
    </tr>
    <tr>
      <td><a href="./conditional/every.html">every</a></td>
      <td>Determina se tutti i valori soddisfano la condizione</td>
    </tr>
    <tr>
      <td><a href="./conditional/isEmpty.html">isEmpty</a></td>
      <td>Determina se non è stato emesso alcun valore</td>
    </tr>
    <!-- Gestione Errori -->
    <tr>
      <th scope="row" rowspan="3"><a href="../error-handling/strategies.html">Gestione Errori</a></th>
      <td><a href="../error-handling/retry-catch.html">catchError</a></td>
      <td>Cattura l'errore ed esegue l'elaborazione di fallback</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retry</a></td>
      <td>Riprova fino al numero specificato di volte in caso di errore</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retryWhen</a></td>
      <td>Riprova con condizioni personalizzate</td>
    </tr>
    <!-- Multicasting -->
    <tr>
      <th scope="row" rowspan="2"><a href="./multicasting/">Multicasting</a></th>
      <td><a href="./multicasting/share.html">share</a></td>
      <td>Condivide l'Observable tra più sottoscrittori</td>
    </tr>
    <tr>
      <td><a href="./multicasting/shareReplay.html">shareReplay</a></td>
      <td>Memorizza gli ultimi N valori e li riproduce per nuovi sottoscrittori</td>
    </tr>
  </tbody>
</table>
