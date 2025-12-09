---
description: RxJS-Operatoren werden nach Kategorien klassifiziert und ihre Funktionen, Anwendungsbereiche und repräsentativen Operatoren werden umfassend in Listenform vorgestellt. Organisiert in sieben Kategorien - Transformation, Filterung, Kombination, Utility, Konditional, Fehlerbehandlung und Multicasting - lernen Sie das Konzept der Pipeline und die praktische Verwendung mit TypeScript.
---

# Verständnis der Operatoren

RxJS-Operatoren sind eine Gruppe von Funktionen zur Transformation, Komposition und Steuerung von Observable-Datenströmen.

Operatoren werden normalerweise in Kombination verwendet, und dabei kommt die "Pipeline" ins Spiel.
- [Was ist die RxJS-Pipeline](./pipeline.md)

In RxJS werden Operatoren in die folgenden Kategorien eingeteilt.


## Kategorieübersicht

- [Transformationsoperatoren](./transformation/)
- [Filterungsoperatoren](./filtering/)
- [Kombinationsoperatoren](./combination/)
- [Utility-Operatoren](./utility/)
- [Konditionaloperatoren](./conditional/)
- [Fehlerbehandlungsoperatoren](../error-handling/strategies.md)
- [Multicasting-Operatoren](./multicasting/)

Jede Kategorie enthält viele nützliche Operatoren.
Details finden Sie in den jeweiligen Kategorien.


## Operatorentabelle

Detaillierte Erklärungen zu jedem Operator finden Sie durch Klicken auf die Links.

<table style="overflow: visible;">
  <caption>
   Übersicht der Operator-Kategorien
  </caption>
  <thead>
    <tr>
      <th scope="col">Kategorie</th>
      <th scope="col">Operator</th>
      <th scope="col">Beschreibung</th>
    </tr>
  </thead>
  <tbody>
    <!-- Transformationsoperatoren -->
    <tr>
      <th scope="row" rowspan="15"><a href="./transformation/">Transformation</a></th>
      <td><a href="./transformation/map.html">map</a></td>
      <td>Transformiert jeden Wert</td>
    </tr>
    <tr>
      <td><a href="./transformation/scan.html">scan</a></td>
      <td>Akkumuliert Werte und gibt auch Zwischenergebnisse aus</td>
    </tr>
    <tr>
      <td><a href="./transformation/reduce.html">reduce</a></td>
      <td>Akkumuliert alle Werte und gibt nur das Endergebnis aus</td>
    </tr>
    <tr>
      <td><a href="./transformation/pairwise.html">pairwise</a></td>
      <td>Verarbeitet zwei aufeinanderfolgende Werte als Paar</td>
    </tr>
    <tr>
      <td><a href="./transformation/groupBy.html">groupBy</a></td>
      <td>Gruppiert Streams nach Schlüsseln</td>
    </tr>
    <tr>
      <td><a href="./transformation/mergeMap.html">mergeMap</a></td>
      <td>Führt asynchrone Verarbeitung parallel aus</td>
    </tr>
    <tr>
      <td><a href="./transformation/switchMap.html">switchMap</a></td>
      <td>Führt nur die neueste asynchrone Verarbeitung aus (alte Verarbeitung wird abgebrochen)</td>
    </tr>
    <tr>
      <td><a href="./transformation/concatMap.html">concatMap</a></td>
      <td>Führt asynchrone Verarbeitung sequenziell aus</td>
    </tr>
    <tr>
      <td><a href="./transformation/exhaustMap.html">exhaustMap</a></td>
      <td>Ignoriert neue Verarbeitung während der Ausführung</td>
    </tr>
    <tr>
      <td><a href="./transformation/expand.html">expand</a></td>
      <td>Erweitert Ergebnisse rekursiv</td>
    </tr>
    <tr>
      <td><a href="./transformation/buffer.html">buffer</a></td>
      <td>Sammelt Werte in einem Array und gibt sie aus</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferTime.html">bufferTime</a></td>
      <td>Sammelt Werte in festgelegten Zeitintervallen</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferCount.html">bufferCount</a></td>
      <td>Sammelt Werte in festgelegten Anzahlen</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferWhen.html">bufferWhen</a></td>
      <td>Buffering mit dynamischer Kontrolle der Endbedingung</td>
    </tr>
    <tr>
      <td><a href="./transformation/bufferToggle.html">bufferToggle</a></td>
      <td>Buffering mit unabhängiger Steuerung von Start und Ende</td>
    </tr>
    <!-- Filterungsoperatoren -->
    <tr>
      <th scope="row" rowspan="22"><a href="./filtering/">Filterung</a></th>
      <td><a href="./filtering/filter.html">filter</a></td>
      <td>Lässt nur Werte durch, die eine Bedingung erfüllen</td>
    </tr>
    <tr>
      <td><a href="./filtering/take.html">take</a></td>
      <td>Nimmt nur die ersten N Werte</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeLast.html">takeLast</a></td>
      <td>Nimmt die letzten N Werte</td>
    </tr>
    <tr>
      <td><a href="./filtering/takeWhile.html">takeWhile</a></td>
      <td>Nimmt Werte, solange eine Bedingung erfüllt ist</td>
    </tr>
    <tr>
      <td><a href="./filtering/skip.html">skip</a></td>
      <td>Überspringt die ersten N Werte</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipLast.html">skipLast</a></td>
      <td>Überspringt die letzten N Werte</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipWhile.html">skipWhile</a></td>
      <td>Überspringt Werte, solange eine Bedingung erfüllt ist</td>
    </tr>
    <tr>
      <td><a href="./filtering/skipUntil.html">skipUntil</a></td>
      <td>Überspringt Werte, bis ein anderes Observable auslöst</td>
    </tr>
    <tr>
      <td><a href="./filtering/first.html">first</a></td>
      <td>Nimmt den ersten Wert oder den ersten Wert, der eine Bedingung erfüllt</td>
    </tr>
    <tr>
      <td><a href="./filtering/last.html">last</a></td>
      <td>Nimmt den letzten Wert oder den letzten Wert, der eine Bedingung erfüllt</td>
    </tr>
    <tr>
      <td><a href="./filtering/elementAt.html">elementAt</a></td>
      <td>Nimmt den Wert am angegebenen Index</td>
    </tr>
    <tr>
      <td><a href="./filtering/find.html">find</a></td>
      <td>Findet den ersten Wert, der eine Bedingung erfüllt</td>
    </tr>
    <tr>
      <td><a href="./filtering/findIndex.html">findIndex</a></td>
      <td>Gibt den Index des ersten Werts zurück, der eine Bedingung erfüllt</td>
    </tr>
    <tr>
      <td><a href="./filtering/debounceTime.html">debounceTime</a></td>
      <td>Gibt den letzten Wert aus, wenn für eine bestimmte Zeit keine Eingabe erfolgt</td>
    </tr>
    <tr>
      <td><a href="./filtering/throttleTime.html">throttleTime</a></td>
      <td>Lässt den ersten Wert durch, ignoriert neue Werte für eine bestimmte Zeit</td>
    </tr>
    <tr>
      <td><a href="./filtering/auditTime.html">auditTime</a></td>
      <td>Gibt den letzten Wert nach einer bestimmten Zeit aus</td>
    </tr>
    <tr>
      <td><a href="./filtering/audit.html">audit</a></td>
      <td>Steuert Zeitraum mit benutzerdefiniertem Observable und gibt letzten Wert aus</td>
    </tr>
    <tr>
      <td><a href="./filtering/sampleTime.html">sampleTime</a></td>
      <td>Sampelt den neuesten Wert in festgelegten Zeitintervallen</td>
    </tr>
    <tr>
      <td><a href="./filtering/ignoreElements.html">ignoreElements</a></td>
      <td>Ignoriert alle Werte, lässt nur Complete/Error durch</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinct.html">distinct</a></td>
      <td>Entfernt alle doppelten Werte (nur eindeutige Werte werden ausgegeben)</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilChanged.html">distinctUntilChanged</a></td>
      <td>Entfernt aufeinanderfolgende doppelte Werte</td>
    </tr>
    <tr>
      <td><a href="./filtering/distinctUntilKeyChanged.html">distinctUntilKeyChanged</a></td>
      <td>Erkennt nur Änderungen einer bestimmten Objekteigenschaft</td>
    </tr>
    <!-- Kombinationsoperatoren (Pipeable) -->
    <tr>
      <th scope="row" rowspan="12"><a href="./combination/">Kombination (Pipeable)</a></th>
      <td><a href="./combination/concatWith.html">concatWith</a></td>
      <td>Kombiniert andere Observables nacheinander nach Abschluss</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeWith.html">mergeWith</a></td>
      <td>Kombiniert mehrere Observables gleichzeitig</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestWith.html">combineLatestWith</a></td>
      <td>Kombiniert die neuesten Werte jedes Observables</td>
    </tr>
    <tr>
      <td><a href="./combination/zipWith.html">zipWith</a></td>
      <td>Paart Werte in entsprechender Reihenfolge</td>
    </tr>
    <tr>
      <td><a href="./combination/raceWith.html">raceWith</a></td>
      <td>Verwendet nur das zuerst auslösende Observable</td>
    </tr>
    <tr>
      <td><a href="./combination/withLatestFrom.html">withLatestFrom</a></td>
      <td>Fügt andere neueste Werte zum Hauptstream hinzu</td>
    </tr>
    <tr>
      <td><a href="./combination/mergeAll.html">mergeAll</a></td>
      <td>Flacht Higher-order Observable parallel ab</td>
    </tr>
    <tr>
      <td><a href="./combination/concatAll.html">concatAll</a></td>
      <td>Flacht Higher-order Observable sequenziell ab</td>
    </tr>
    <tr>
      <td><a href="./combination/switchAll.html">switchAll</a></td>
      <td>Wechselt zum neuesten Higher-order Observable</td>
    </tr>
    <tr>
      <td><a href="./combination/exhaustAll.html">exhaustAll</a></td>
      <td>Ignoriert neue Higher-order Observables während der Ausführung</td>
    </tr>
    <tr>
      <td><a href="./combination/combineLatestAll.html">combineLatestAll</a></td>
      <td>Kombiniert die neuesten Werte aller inneren Observables</td>
    </tr>
    <tr>
      <td><a href="./combination/zipAll.html">zipAll</a></td>
      <td>Paart entsprechende Werte jedes inneren Observables</td>
    </tr>
    <!-- Utility-Operatoren -->
    <tr>
      <th scope="row" rowspan="15"><a href="./utility/">Utility</a></th>
      <td><a href="./utility/tap.html">tap</a></td>
      <td>Führt Seiteneffekte aus (z.B. Log-Ausgabe)</td>
    </tr>
    <tr>
      <td><a href="./utility/finalize.html">finalize</a></td>
      <td>Führt Nachverarbeitung bei Complete oder Error aus</td>
    </tr>
    <tr>
      <td><a href="./utility/delay.html">delay</a></td>
      <td>Verzögert alle Werte um eine bestimmte Zeit</td>
    </tr>
    <tr>
      <td><a href="./utility/delayWhen.html">delayWhen</a></td>
      <td>Verzögert jeden Wert dynamisch mit einem anderen Observable</td>
    </tr>
    <tr>
      <td><a href="./utility/timeout.html">timeout</a></td>
      <td>Gibt einen Fehler aus, wenn innerhalb einer bestimmten Zeit kein Wert kommt</td>
    </tr>
    <tr>
      <td><a href="./utility/takeUntil.html">takeUntil</a></td>
      <td>Nimmt Werte, bis ein anderes Observable einen Wert ausgibt</td>
    </tr>
    <tr>
      <td><a href="./utility/retry.html">retry</a></td>
      <td>Wiederholt bei Fehler bis zu einer bestimmten Anzahl</td>
    </tr>
    <tr>
      <td><a href="./utility/repeat.html">repeat</a></td>
      <td>Wiederholt nach Abschluss eine bestimmte Anzahl von Malen</td>
    </tr>
    <tr>
      <td><a href="./utility/startWith.html">startWith</a></td>
      <td>Fügt am Anfang des Streams einen Anfangswert hinzu</td>
    </tr>
    <tr>
      <td><a href="./utility/toArray.html">toArray</a></td>
      <td>Sammelt alle Werte in einem Array und gibt sie aus</td>
    </tr>
    <tr>
      <td><a href="./utility/materialize.html">materialize</a></td>
      <td>Konvertiert Benachrichtigungen in Notification-Objekte</td>
    </tr>
    <tr>
      <td><a href="./utility/dematerialize.html">dematerialize</a></td>
      <td>Konvertiert Notification-Objekte zurück in normale Benachrichtigungen</td>
    </tr>
    <tr>
      <td><a href="./utility/observeOn.html">observeOn</a></td>
      <td>Steuert das Timing der Werteausgabe mit einem Scheduler</td>
    </tr>
    <tr>
      <td><a href="./utility/subscribeOn.html">subscribeOn</a></td>
      <td>Steuert das Timing des Subscription-Starts mit einem Scheduler</td>
    </tr>
    <tr>
      <td><a href="./utility/timestamp.html">timestamp</a></td>
      <td>Fügt jedem Wert einen Zeitstempel hinzu</td>
    </tr>
    <!-- Konditionaloperatoren -->
    <tr>
      <th scope="row" rowspan="3"><a href="./conditional/">Konditional</a></th>
      <td><a href="./conditional/defaultIfEmpty.html">defaultIfEmpty</a></td>
      <td>Gibt einen Standardwert aus, wenn keine Werte vorhanden sind</td>
    </tr>
    <tr>
      <td><a href="./conditional/every.html">every</a></td>
      <td>Prüft, ob alle Werte eine Bedingung erfüllen</td>
    </tr>
    <tr>
      <td><a href="./conditional/isEmpty.html">isEmpty</a></td>
      <td>Prüft, ob keine Werte ausgegeben wurden</td>
    </tr>
    <!-- Fehlerbehandlung -->
    <tr>
      <th scope="row" rowspan="3"><a href="../error-handling/strategies.html">Fehlerbehandlung</a></th>
      <td><a href="../error-handling/retry-catch.html">catchError</a></td>
      <td>Fängt Fehler ab und führt Fallback-Verarbeitung aus</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retry</a></td>
      <td>Wiederholt bei Fehler bis zu einer bestimmten Anzahl</td>
    </tr>
    <tr>
      <td><a href="../error-handling/retry-catch.html">retryWhen</a></td>
      <td>Wiederholt mit benutzerdefinierten Bedingungen</td>
    </tr>
    <!-- Multicast -->
    <tr>
      <th scope="row" rowspan="2"><a href="./multicasting/">Multicast</a></th>
      <td><a href="./multicasting/share.html">share</a></td>
      <td>Teilt ein Observable zwischen mehreren Subscribern</td>
    </tr>
    <tr>
      <td><a href="./multicasting/shareReplay.html">shareReplay</a></td>
      <td>Cached die neuesten N Werte und spielt sie für neue Subscriber ab</td>
    </tr>
  </tbody>
</table>
