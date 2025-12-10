---
description: "Spiega sistematicamente come creare Observable in RxJS, dalle funzioni di generazione di base come of e from, alla definizione di Observable personalizzati, allo streaming di comunicazioni HTTP e alla conversione di eventi in Observable. Presenta pattern di implementazione che sfruttano l'inferenza di tipo di TypeScript."
---
# Come creare un Observable

Un Observable definisce un "flusso di dati" e ci sono molti modi diversi per crearne uno.
RxJS fornisce una serie di mezzi per creare Observable personalizzati o per generare facilmente Observable da eventi, array, risposte HTTP, ecc.

Questa sezione fornisce una panoramica completa su come creare Observable in RxJS, dalla sintassi di base alle applicazioni pratiche.

## Classificazione dei metodi di creazione degli Observable

Di seguito è riportato un elenco categorizzato delle principali tecniche di creazione.

| Categoria | Metodi principali | Descrizione |
|----------|----------|------|
| Creazione personalizzata | [`new Observable()`](#new-observable) | Molto flessibile, ma anche molto descrittivo. È necessaria una pulizia manuale |
| Creation Functions | [`of()`](#of), [`from()`](#from), [`fromEvent()`](#fromevent), [`interval()`](#interval-timer), [`timer()`](#interval-timer), [`ajax()`](#ajax), [`fromFetch()`](#fromfetch), [`scheduled()`](#scheduled) | Funzioni di creazione basate su dati, eventi e tempo comunemente utilizzate |
| Creation Functions speciali | [`defer()`](#defer), [`range()`](#range), [`generate()`](#generate), [`iif()`](#iif) | Generazione controllata e a ciclo, commutazione condizionale, ecc. |
| Observable speciali | [`EMPTY`](#empty-never-throwerror), [`NEVER`](#empty-never-throwerror), [`throwError()`](#empty-never-throwerror) | Per completamento, non fare nulla, emettere errore |
| Sistema Subject | [`Subject`](#subject-behaviorsubject), [`BehaviorSubject`](#subject-behaviorsubject) | Observable speciale che può agire sia come osservatore che come mittente |
| Conversione callback | [`bindCallback()`](#bindcallback), [`bindNodeCallback()`](#bindnodecallback) | Conversione di funzioni basate su callback in Observable |
| Controllo risorse | [`using()`](#using) | Sottoscrivere l'Observable e controllare le risorse allo stesso tempo |
| WebSocket | [`webSocket()`](#websocket) | Trattare la comunicazione WebSocket come Observable bidirezionale |



## Creazione personalizzata

### new Observable()
[📘 RxJS Official: Observable](https://rxjs.dev/api/index/class/Observable)


Il metodo più semplice consiste nell'utilizzare direttamente il costruttore `Observable`. Questo metodo è più flessibile quando si vuole definire una logica Observable personalizzata. Le chiamate esplicite `next`, `error` e `complete` consentono un controllo del comportamento a grana fine.

```ts
import { Observable } from 'rxjs';

const observable$ = new Observable<number>(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  setTimeout(() => {
    subscriber.next(4);
    subscriber.complete();
  }, 1000);
});

observable$.subscribe({
  next: value => console.log('Valore:', value),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});
// Output:
// Valore: 1
// Valore: 2
// Valore: 3
// Valore: 4
// Completato
```

> [!CAUTION]
> Se si usa `new Observable()`, è necessario scrivere un rilascio esplicito delle risorse (processo di pulizia).
> ```ts
> const obs$ = new Observable(subscriber => {
>   const id = setInterval(() => subscriber.next(Date.now()), 1000);
>   return () => {
>     clearInterval(id); // rilascio esplicito della risorsa
>   };
> });
> ```
> D'altra parte, le funzioni di creazione integrate in RxJS, come `fromEvent()` e `interval()`, hanno un processo di pulizia adeguato internamente.
> ```ts
> const click$ = fromEvent(document, 'click');
> const timer$ = interval(1000);
> ```
> Questi usano internamente `addEventListener` e `setInterval` e sono progettati in modo che RxJS chiami automaticamente `removeEventListener` e `clearInterval()` quando viene chiamato `unsubscribe()`.
>
> Si noti che anche se il processo di pulizia è implementato all'interno di RxJS, non verrà eseguito a meno che non venga chiamato `unsubscribe()`.
> ```ts
>  const subscription = observable$.subscribe({
>  // omesso...
>  });
>
>  subscription.unsubscribe(); // 👈
> ```
> - Qualunque sia il modo in cui si crea l'Observable, è bene abituarsi a chiamare sempre `unsubscribe()` quando non è più necessario.
> - Dimenticando di annullare l'iscrizione, gli event listener e i timer rimarranno in funzione, causando memory leak ed effetti collaterali inaspettati.


## Creation Functions (Funzioni di creazione)

Per una creazione di Observable più concisa e specifica, sono utili le "Creation Functions" fornite da RxJS. Esse semplificano il codice per casi d'uso ripetuti.

> [!NOTE]
> Queste sono classificate come "Creation Functions (funzioni di creazione)" nella documentazione ufficiale di RxJS.
> In precedenza (RxJS 5.x ~ 6) erano chiamate "creation operator (operatori di creazione)", ma da RxJS 7 in poi il termine ufficiale è "Creation Functions".


### of()
[📘 RxJS Official: of()](https://rxjs.dev/api/index/function/of)

La più semplice funzione di creazione di un Observable che emette più valori **uno alla volta** in sequenza.


```ts
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Valore:', value),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});
// Output: Valore: 1, Valore: 2, Valore: 3, Valore: 4, Valore: 5, Completato
```

> [!IMPORTANT]
> Differenza tra `of()` e `from()`
> - `of([1, 2, 3])` → emette un array.
> - `from([1, 2, 3])` → emette i singoli valori `1`, `2`, `3` in sequenza.
>
> È facile confonderli, quindi fate attenzione.

> [!TIP]
> Per un uso dettagliato ed esempi pratici, vedere la [pagina dettagliata di of()](/it/guide/creation-functions/basic/of).

### from()
[📘 RxJS Official: from()](https://rxjs.dev/api/index/function/from)

Genera un Observable da **strutture di dati esistenti**, come array, Promise, iterabili, ecc.

```ts
import { from } from 'rxjs';

// Creato da array
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Valore array:', value),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});

// Creato da Promise
const promise$ = from(Promise.resolve('Risultato Promise'));
promise$.subscribe({
  next: value => console.log('Risultato Promise:', value),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});

// Creato da iterabile
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Valore iterabile:', value),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});

// Output:
// Valore array: 1
// Valore array: 2
// Valore array: 3
// Completato
// Valore iterabile: 1
// Valore iterabile: 2
// Valore iterabile: 3
// Completato
// Risultato Promise: Risultato Promise
// Completato
```

> [!TIP]
> Per un uso dettagliato ed esempi pratici, vedere la [pagina dettagliata di from()](/it/guide/creation-functions/basic/from).

### fromEvent()
[📘 RxJS Official: fromEvent](https://rxjs.dev/api/index/function/fromEvent)

Funzione per **gestire sorgenti di eventi come Observable**, ad esempio eventi DOM.

```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe({
  next: event => console.log('Evento click:', event),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});

// Output:
// Evento click: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!CAUTION]
> Notare i target degli eventi supportati
> - `fromEvent()` supporta elementi DOM del browser (implementazione EventTarget), EventEmitter di Node.js e target di eventi simili a jQuery.
> - Sottoscrizioni multiple possono aggiungere più event listener.

> 👉 Per esempi più dettagliati di utilizzo dei flussi di eventi, vedere [Streaming degli eventi](../observables/events).

> [!TIP]
> Per un uso dettagliato ed esempi pratici, vedere la [pagina dettagliata di fromEvent()](/it/guide/creation-functions/basic/fromEvent).

### interval(), timer()
[📘 RxJS Official: interval](https://rxjs.dev/api/index/function/interval), [📘 RxJS Official: timer](https://rxjs.dev/api/index/function/timer)

Si usano quando si desidera emettere valori in modo continuo a intervalli regolari o **quando è necessario un controllo temporale**.

```ts
import { interval, timer } from 'rxjs';

// Emette un valore ogni secondo
const interval$ = interval(1000);
interval$.subscribe({
  next: value => console.log('Intervallo:', value),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});

// Si avvia dopo 3 secondi e successivamente emette un valore ogni secondo
const timer$ = timer(3000, 1000);
timer$.subscribe({
  next: value => console.log('Timer:', value),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});

// Output:
// Intervallo: 0
// Intervallo: 1
// Intervallo: 2
// Timer: 0
// Intervallo: 3
// Timer: 1
// Intervallo: 4
// Timer: 2
// .
// .
```
Le funzioni `interval()` e `timer()` sono utilizzate frequentemente nelle operazioni relative al controllo del tempo e sono particolarmente adatte per animazioni, polling e ritardi di eventi asincroni.

> [!CAUTION]
> Si noti che sono Cold Observable
> - `interval()` e `timer()` sono Cold Observable e vengono eseguiti indipendentemente per ogni sottoscrizione.
> - Se necessario, si può pensare di renderli Hot, ad esempio con `share()`.
>
> Per maggiori informazioni, vedere "[Observable Cold e Observable Hot](./cold-and-hot-observables.md)".

> [!TIP]
> Per un uso dettagliato ed esempi pratici, vedere la [pagina dettagliata di interval()](/it/guide/creation-functions/basic/interval) e la [pagina dettagliata di timer()](/it/guide/creation-functions/basic/timer).

### ajax()
[📘 RxJS Official: ajax](https://rxjs.dev/api/ajax/ajax)

Funzione per la gestione **asincrona** del risultato di una comunicazione HTTP come **Observable**.

```ts
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
api$.subscribe({
  next: response => console.log('Risposta API:', response),
  error: error => console.error('Errore API:', error),
  complete: () => console.log('API completata')
});

// Output:
// Risposta API: {userId: 1, id: 1, title: 'delectus aut autem', completed: false}
// API completata
```

> [!NOTE]
> RxJS ajax utilizza internamente XMLHttpRequest. D'altra parte, RxJS ha anche un operatore chiamato fromFetch, che utilizza l'API Fetch per effettuare richieste HTTP.

> [!TIP]
> Per un uso dettagliato ed esempi pratici, vedere la [pagina dettagliata di ajax()](/it/guide/creation-functions/http-communication/ajax). Per una panoramica della comunicazione HTTP, vedere [Creation Functions per comunicazione HTTP](/it/guide/creation-functions/http-communication/).

### fromFetch()
[📘 RxJS Official: fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` è una funzione che avvolge l'API Fetch e consente di trattare le richieste HTTP come Observable.
È simile ad `ajax()`, ma è più moderna e leggera.

```ts
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs';

const api$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1');

api$.pipe(
  switchMap(response => response.json())
).subscribe({
  next: data => console.log('Dati:', data),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});

// Output:
// Dati: Objectcompleted: falseid: 1title: "delectus aut autem"userId: 1[[Prototype]]: Object
// Completato
```

> [!NOTE]
> `fromFetch()` utilizza l'API Fetch, quindi, a differenza di `ajax()`, l'inizializzazione della configurazione della richiesta e la conversione `.json()` della risposta devono essere effettuate manualmente.
> Anche la gestione degli errori e i controlli dello stato HTTP devono essere eseguiti correttamente.

> [!TIP]
> Per un uso dettagliato ed esempi pratici, vedere la [pagina dettagliata di fromFetch()](/it/guide/creation-functions/http-communication/fromFetch). Per una panoramica della comunicazione HTTP, vedere [Creation Functions per comunicazione HTTP](/it/guide/creation-functions/http-communication/).

### scheduled()
[📘 RxJS Official: scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` è una funzione che consente di specificare esplicitamente uno scheduler per funzioni di emissione come `of()` e `from()`.
Si usa quando si vuole controllare in dettaglio la tempistica dell'esecuzione sincrona o asincrona.

```ts
import { scheduled, asyncScheduler } from 'rxjs';

const observable$ = scheduled([1, 2, 3], asyncScheduler);
observable$.subscribe({
  next: val => console.log('Valore:', val),
  complete: () => console.log('Completato')
});

// L'esecuzione è asincrona
// Output:
// Valore: 1
// Valore: 2
// Valore: 3
// Completato
```

> [!NOTE]
> `scheduled()` consente alle funzioni sincrone esistenti (ad esempio `of()`, `from()`) di lavorare in modo asincrono.
> Questo è utile per i test e per l'ottimizzazione delle prestazioni dell'interfaccia utente, quando è richiesto un controllo asincrono dell'elaborazione.

> [!TIP]
> Per un uso dettagliato ed esempi pratici, vedere la [pagina dettagliata di scheduled()](/it/guide/creation-functions/control/scheduled). Per una panoramica del sistema di controllo, vedere [Creation Functions di controllo](/it/guide/creation-functions/control/).

### defer()
[📘 RxJS Official: defer](https://rxjs.dev/api/index/function/defer)

Si usa quando si vuole ritardare la generazione di un Observable fino al **momento della sottoscrizione**.

```ts
import { defer, of } from 'rxjs';

const deferred$ = defer(() => {
  const randomValue = Math.random();
  return randomValue > 0.5 ?
    of('Valore maggiore del 50%:', randomValue) :
    of('Valore minore o uguale al 50%:', randomValue);
});

// Viene creato un nuovo Observable per ogni sottoscrizione
deferred$.subscribe(value => console.log(value));
deferred$.subscribe(value => console.log(value));

// Output:
// Valore minore o uguale al 50%:
// 0.08011364416212319
// Valore minore o uguale al 50%:
// 0.3141403962502316
```
`defer()` è utile quando si vuole ritardare l'elaborazione con effetti collaterali al momento della sottoscrizione piuttosto che alla creazione dell'Observable. È adatto per applicazioni come la generazione casuale o l'ottenimento dell'ora corrente.

> [!IMPORTANT]
> Differenza da `of()`
> - `of()` ha un valore fisso al momento della creazione.
> - `defer()` viene elaborato per la prima volta al momento della sottoscrizione, il che lo rende adatto per elaborazioni in cui il valore cambia a ogni sottoscrizione.

### range()
[📘 RxJS Official: range](https://rxjs.dev/api/index/function/range)

Crea un Observable che emette una serie di numeri all'interno di un intervallo specificato.

```ts
import { range } from 'rxjs';

const range$ = range(5, 3); // 3 numeri a partire da 5 → 5, 6, 7
range$.subscribe({
  next: val => console.log('range:', val),
  complete: () => console.log('Completato')
});

// Output:
// range: 5
// range: 6
// range: 7
// Completato
```

### generate()
[📘 RxJS Official: generate](https://rxjs.dev/api/index/function/generate)

Funzione per **generare numeri e stati** come un ciclo, specificando valori iniziali, condizioni ed espressioni di aggiornamento.

```ts
import { generate } from 'rxjs';

const generate$ = generate({
  initialState: 0,
  condition: x => x < 5,
  iterate: x => x + 1
});

generate$.subscribe({
  next: val => console.log('generate:', val),
  complete: () => console.log('Completato')
});

// Output:
// generate: 0
// generate: 1
// generate: 2
// generate: 3
// generate: 4
// Completato
```

### iif()
[📘 RxJS Official: iif](https://rxjs.dev/api/index/function/iif)

Funzione per **cambiare dinamicamente l'Observable da eseguire** in base alla condizione.

```ts
import { iif, of, EMPTY } from 'rxjs';

const condition = true;
const iif$ = iif(() => condition, of('La condizione è true'), EMPTY);

iif$.subscribe({
  next: val => console.log('iif:', val),
  complete: () => console.log('Completato')
});

// Output:
// iif: La condizione è true
// Completato
```

> [!NOTE]
> `iif()` può cambiare dinamicamente l'Observable restituito a seconda della condizione. Questo è utile per il controllo del flusso.

## Observable speciali

### EMPTY, NEVER, throwError()
[📘 RxJS Official: EMPTY](https://rxjs.dev/api/index/const/EMPTY), [📘 RxJS Official: NEVER](https://rxjs.dev/api/index/const/NEVER), [📘 RxJS Official: throwError](https://rxjs.dev/api/index/function/throwError)


In RxJS sono disponibili anche Observable speciali utili per il controllo dell'esecuzione, la gestione delle eccezioni e per scopi didattici.

```ts
import { EMPTY, throwError, NEVER } from 'rxjs';

// Observable che completa immediatamente
const empty$ = EMPTY;
empty$.subscribe({
  next: () => console.log('Questo non viene visualizzato'),
  complete: () => console.log('Completato immediatamente')
});

// Observable che emette un errore
const error$ = throwError(() => new Error('Si è verificato un errore'));
error$.subscribe({
  next: () => console.log('Questo non viene visualizzato'),
  error: err => console.error('Errore:', err.message),
  complete: () => console.log('Completato')
});

// Observable che non emette nulla e non completa
const never$ = NEVER;
never$.subscribe({
  next: () => console.log('Questo non viene visualizzato'),
  complete: () => console.log('Anche questo non viene visualizzato')
});

// Output:
// Completato immediatamente
// Errore: Si è verificato un errore
```

> [!IMPORTANT]
> Principalmente per scopi di controllo, verifica e apprendimento
> - `EMPTY`, `NEVER` e `throwError()` sono utilizzati per **controllo del flusso, verifica della gestione delle eccezioni** o scopi didattici, piuttosto che per normali flussi di dati.


## Sistema Subject

### Subject, BehaviorSubject, ecc. {#subject-behaviorsubject}
[📘 RxJS Official: Subject](https://rxjs.dev/api/index/class/Subject), [📘 RxJS Official: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Observable che può emettere i propri valori, adatto per **multicasting e condivisione dello stato**.

```ts
import { Subject } from 'rxjs';

const subject$ = new Subject<number>();

// Utilizzare come Observer
subject$.subscribe(value => console.log('Observer 1:', value));
subject$.subscribe(value => console.log('Observer 2:', value));

// Utilizzare come Observable
subject$.next(1);
subject$.next(2);
subject$.next(3);
subject$.complete();

// Output:
// Observer 1: 1
// Observer 2: 1
// Observer 1: 2
// Observer 2: 2
// Observer 1: 3
// Observer 2: 3
```

> [!IMPORTANT]
> Si noti che è un Hot Observable
> - Il `Subject` notifica i sottoscrittori "simultaneamente", quindi, a differenza dei Cold Observable come `from()` e `of()`, **potrebbe non ricevere il valore a seconda dei tempi di sottoscrizione**.

Per maggiori informazioni, vedere ["Cos'è un Subject"](../subjects/what-is-subject.md).


## Conversione callback

RxJS fornisce le funzioni `bindCallback()` e `bindNodeCallback()` per convertire le funzioni asincrone basate su callback in Observable.

### bindCallback()
[📘 RxJS Official: bindCallback](https://rxjs.dev/api/index/function/bindCallback)

`bindCallback()` converte una funzione asincrona il cui "ultimo argomento è una funzione di callback" in una funzione che restituisce un Observable.

```ts
import { bindCallback } from 'rxjs';

// Funzione asincrona in formato callback
function asyncFn(input: string, callback: (result: string) => void) {
  setTimeout(() => callback(`Hello, ${input}`), 1000);
}

// Convertire asyncFn in "funzione che restituisce Observable"
const observableFn = bindCallback(asyncFn);
const result$ = observableFn('RxJS');

result$.subscribe({
  next: val => console.log(val), // Hello, RxJS
  complete: () => console.log('Completato')
});

// Output:
// Hello, RxJS
// Completato
```

### bindNodeCallback()
[📘 RxJS Official: bindNodeCallback](https://rxjs.dev/api/index/function/bindNodeCallback)

`bindNodeCallback()` converte una funzione Node.js in formato "error-first callback (err, result)" in una funzione che restituisce un Observable.

```ts
import { bindNodeCallback } from 'rxjs';
import { readFile } from 'fs';

// Convertire readFile in una "funzione che restituisce Observable" (gli errori sono notificati tramite error)
const readFile$ = bindNodeCallback(readFile);
readFile$('./some.txt').subscribe({
  next: data => console.log('Contenuto:', data),
  error: err => console.error('Errore:', err)
});
```

> [!NOTE]
> `bindNodeCallback()` supporta funzioni asincrone di tipo `(err, result)` in Node.js.

### Differenza tra bindCallback() e bindNodeCallback()
La differenza tra bindCallback() e bindNodeCallback() è la forma della funzione callback di destinazione.

| Funzione | Formato della funzione di destinazione | Caratteristica |
|---|---|---|
| bindCallback() | callback(result) | corrisponde a un normale callback (un argomento) |
| bindNodeCallback() | callback(error, result) | corrisponde al formato error-first di Node.js |

#### Esempio concreto: target di bindCallback()

```ts
function doSomething(input: string, callback: (result: string) => void) {
  callback(`Risultato: ${input}`);
}
```
→ bindCallback() può essere usato per convertire


#### Esempio concreto: target di bindNodeCallback() (stile Node.js)

```ts
function readFile(path: string, cb: (err: Error | null, data: string) => void) {
  if (path === 'valid.txt') cb(null, 'file content');
  else cb(new Error('not found'), '');
}
```
→ bindNodeCallback() può essere usato per notificare l'errore come Observable quando si verifica un errore.


> [!NOTE]
> Uso
> - bindNodeCallback() se il primo argomento del callback è "errore o meno"
> - bindCallback() per un semplice callback "che restituisce solo un valore"

## Controllo risorse

### using()
[📘 RxJS Official: using](https://rxjs.dev/api/index/function/using)

`using()` è una funzione che associa la creazione e il rilascio di risorse al ciclo di vita di un Observable.
È utile in combinazione con **processi che richiedono una pulizia manuale**, come WebSocket, event listener e risorse esterne.

```ts
import { using, interval, Subscription } from 'rxjs';

const resource$ = using(
  () => new Subscription(() => console.log('Risorsa rilasciata')),
  () => interval(1000)
);

const sub = resource$.subscribe(value => console.log('Valore:', value));

// Disiscrizione dopo alcuni secondi
setTimeout(() => sub.unsubscribe(), 3500);

// Output:
// Valore: 0
// Valore: 1
// Valore: 2
// Risorsa rilasciata
```

> [!IMPORTANT]
> `using()` è utile per far coincidere l'ambito della risorsa con la sottoscrizione dell'Observable.
> Il processo di pulizia esplicita viene richiamato automaticamente quando viene chiamato `unsubscribe()`.

> [!TIP]
> Per un uso dettagliato ed esempi pratici, vedere la [pagina dettagliata di using()](/it/guide/creation-functions/control/using). Per una panoramica del sistema di controllo, vedere [Creation Functions di controllo](/it/guide/creation-functions/control/).

## WebSocket()
[📘 RxJS Official: webSocket](https://rxjs.dev/api/webSocket/webSocket)

Il modulo `rxjs/webSocket` di RxJS fornisce una funzione `webSocket()` che può gestire i WebSocket come Observable/Observer.

```ts
import { webSocket } from 'rxjs/webSocket';

const socket$ = webSocket('wss://echo.websocket.org');

socket$.subscribe({
  next: msg => console.log('Ricevuto:', msg),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});

// Invio di messaggi (come Observer)
socket$.next('Hello WebSocket!');
```

> [!IMPORTANT]
> `webSocket()` è un ibrido Observable/Observer in grado di comunicare bidirezionalmente.
> È utile per le comunicazioni in tempo reale, in quanto può gestire facilmente le connessioni WebSocket, invio e ricezione come Observable.


## Riepilogo

Gli stream RxJS consentono di gestire la tradizionale gestione degli eventi JavaScript e la comunicazione AJAX in un'interfaccia unificata. È particolarmente potente quando si tratta di dati variabili nel tempo e quando si combinano più sorgenti di eventi.
