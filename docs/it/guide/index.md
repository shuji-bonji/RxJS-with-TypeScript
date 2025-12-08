---
description: "Una guida all'apprendimento per studiare sistematicamente RxJS in un ambiente TypeScript. Fornisce spiegazioni pratiche, passo dopo passo, che coprono tutto, dai fondamenti degli Osservabili agli Oggetti, ai vari operatori, alla gestione degli errori, agli schedulatori e alle tecniche di test."
---

# Guida

Questa guida aiuta a imparare sistematicamente RxJS in un ambiente TypeScript.
Procedendo in ordine attraverso le sezioni che seguono, è possibile acquisire una comprensione strutturata di RxJS, dai fondamenti ai concetti avanzati.

## Indice

### 1. Introduzione a RxJS
- [Come iniziare](/it/guide/introduction)
- [Impostazione dell'ambiente di apprendimento](/it/guide/starter-kid)
- [Che cos'è RxJS?](/it/guide/basics/what-is-rxjs)
- [Cos'è uno stream?](/it/guide/basics/what-is-a-stream)
- [Promise vs. RxJS](/it/guide/basics/promise-vs-rxjs)

### 2. Fondamenti degli osservabili
- [Che cos'è un osservabile?](/it/guide/observables/what-is-observable)
- [Come creare un osservabile](/it/guide/observables/creation)
- [Eventi in streaming](/it/guide/observables/events)
- [Elenco di eventi](/it/guide/observables/events-list)
- [Observer vs Subscriber](/it/guide/observables/observer-vs-subscriber)
- [Ciclo di vita degli osservabili](/it/guide/observables/observable-lifecycle)
- [Osservabili freddi e osservabili caldi](/it/guide/observables/cold-and-hot-observables)

### 3. Funzioni di creazione
- [Cosa sono le funzioni di creazione?](/it/guide/creation-functions/)
- [Funzioni di creazione di base](/it/guide/creation-functions/basic/) - of, from, fromEvent, interval, timer
- [Funzioni di generazione di loop](/it/guide/creation-functions/loop/) - range, generate
- [Funzioni di comunicazione HTTP](/it/guide/creation-functions/http-communication/) - ajax, fromFetch
- [Funzioni di combinazione](/it/guide/creation-functions/combination/) - concat, merge, combineLatest, zip, forkJoin
- [Funzioni di selezione e partizione](/it/guide/creation-functions/selection/) - race, partition
- [Funzioni condizionali](/it/guide/creation-functions/conditional/) - iif, defer
- [Funzioni di controllo](/it/guide/creation-functions/control/) - scheduled, using

### 4. Comprendere gli operatori
- [Panoramica degli operatori](/it/guide/operators/)
- [Concetti di pipeline](/it/guide/operators/pipeline)
- [Operatori di trasformazione](/it/guide/operators/transformation/) - map, scan, mergeMap, switchMap, etc.
- [Operatori di filtraggio](/it/guide/operators/filtering/) - filter, take, debounceTime, throttleTime, etc.
- [Operatori di combinazione](/it/guide/operators/combination/) - withLatestFrom, mergeWith, etc.
- [Operatori di utilità](/it/guide/operators/utility/) - tap, delay, retry, finalize, takeUntil, etc.
- [Multicasting](/it/guide/operators/multicasting/) - share, shareReplay, etc.

### 5. Soggetti e multicasting
- [Che cos'è un Subject?](/it/guide/subjects/what-is-subject)
- [Tipi di Subject](/it/guide/subjects/types-of-subject)
- [Come funziona il multicasting](/it/guide/subjects/multicasting)
- [Casi d'uso dei Subject](/it/guide/subjects/use-cases)

### 6. Gestione degli errori
- [Strategie di gestione degli errori](/it/guide/error-handling/strategies)
- [Due posizioni per la gestione degli errori](/it/guide/error-handling/error-handling-locations)
- [Integrazione di try-catch con RxJS](/it/guide/error-handling/try-catch-integration)
- [retry e catchError](/it/guide/error-handling/retry-catch)
- [finalize e complete](/it/guide/error-handling/finalize)

### 7. Utilizzare gli schedulatori
- [Controllare l'elaborazione asincrona](/it/guide/schedulers/async-control)
- [Tipi di scheduler e loro utilizzo](/it/guide/schedulers/types)
- [Nozioni di base su task e scheduler](/it/guide/schedulers/task-and-scheduler-basics)

### 8. Tecniche di debug di RxJS
- [Panoramica delle tecniche di debug](/it/guide/debugging/)
- [Scenari di debug comuni](/it/guide/debugging/common-scenarios)
- [Strumenti di debug personalizzati](/it/guide/debugging/custom-tools)
- [Debug delle prestazioni](/it/guide/debugging/performance)

### 9. Tecniche di test
- [Test unitari di RxJS](/it/guide/testing/unit-tests)
- [Usare TestScheduler](/it/guide/testing/test-scheduler)
- [Marble Testing](/it/guide/testing/marble-testing)

### 10. Raccolta di anti-pattern di RxJS
- [Cosa sono gli anti-pattern?](/it/guide/anti-patterns/)
- [Errori comuni e soluzioni](/it/guide/anti-patterns/common-mistakes)
- [Proliferazione dei flag](/it/guide/anti-patterns/flag-management)
- [Dichiarazioni if annidate nelle sottoscrizioni](/it/guide/anti-patterns/subscribe-if-hell)
- [Mescolare promesse e osservabili](/it/guide/anti-patterns/promise-observable-mixing)
- [L'inferno di una riga](/it/guide/anti-patterns/one-liner-hell)
- [Lista di controllo per evitare gli anti-pattern](/it/guide/anti-patterns/checklist)

### 11. Superare le difficoltà di RxJS
- [Perché RxJS è difficile](/it/guide/overcoming-difficulties/)
- [Barriera della comprensione concettuale](/it/guide/overcoming-difficulties/conceptual-understanding)
- [L'ostacolo della gestione del ciclo di vita](/it/guide/overcoming-difficulties/lifecycle-management)
- [Dilemmi nella selezione degli operatori](/it/guide/overcoming-difficulties/operator-selection)
- [Comprensione di tempi e ordini](/it/guide/overcoming-difficulties/timing-and-order)
- [Difficoltà con la gestione degli stati](/it/guide/overcoming-difficulties/state-and-sharing)
- [Combinazione di più flussi](/it/guide/overcoming-difficulties/stream-combination)
- [Sfide del debug](/it/guide/overcoming-difficulties/debugging-guide)

### 13. Raccolta di modelli pratici
- [Panoramica dei modelli pratici](/it/guide/practical-patterns/)
- [Gestione degli eventi dell'interfaccia utente](/it/guide/practical-patterns/ui-events)
- [Chiamate API](/it/guide/practical-patterns/api-calls)
- [Gestione dei moduli](/it/guide/practical-patterns/form-handling)
- [Pattern form avanzati](/it/guide/practical-patterns/advanced-form-patterns)
- [Elaborazione dei dati in tempo reale](/it/guide/practical-patterns/real-time-data)
- [Strategie di caching](/it/guide/practical-patterns/caching-strategies)
- [Modelli di gestione degli errori](/it/guide/practical-patterns/error-handling-patterns)
- [Diramazione condizionale nelle sottoscrizioni](/it/guide/practical-patterns/subscribe-branching)

### Appendice
- [Panoramica dell'appendice](/it/guide/appendix/)
- [Sviluppo incorporato e programmazione reattiva](/it/guide/appendix/embedded-reactive-programming)
- [Modelli reattivi oltre RxJS](/it/guide/appendix/reactive-patterns-beyond-rxjs)
- [Mappa dell'architettura reattiva](/it/guide/appendix/reactive-architecture-map)
- [Programmazione reattiva riconsiderata](/it/guide/appendix/reactive-programming-reconsidered)
- [Ecosistema RxJS e Reactive Streams](/it/guide/appendix/rxjs-and-reactive-streams-ecosystem)

---

> [!NOTE]
> Questa guida è strutturata in modo da approfondire la comprensione di RxJS in modo graduale e sistematico. Sentitevi liberi di fare riferimento a qualsiasi sezione, se necessario.
