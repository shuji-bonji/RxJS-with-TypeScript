---
description: "Een systematische uitleg van hoe Observables in RxJS te maken, van basisfuncties voor aanmaak zoals of en from, tot het definiëren van aangepaste Observables, het stroomvormig maken van HTTP-communicatie en het omzetten van events naar Observables. Introduceert implementatiepatronen met TypeScript type-inferentie."
---
# Hoe Observables Maken

Observable is iets dat "datastromen" definieert, en er zijn verschillende manieren om ze te maken.
RxJS biedt verschillende methoden om eenvoudig Observables te maken van aangepaste Observables, events, arrays, HTTP-responses, enzovoort.

Hier introduceren we uitgebreid de manieren om Observables in RxJS te maken, van basale syntaxis tot praktische toepassingen.

## Classificatie van Observable-aanmaakmethoden

Hieronder volgt een overzicht per categorie van de belangrijkste aanmaakmethoden.

| Categorie | Belangrijkste Methoden | Uitleg |
|----------|----------|------|
| Aangepaste creatie | [`new Observable()`](#new-observable) | Hoge vrijheid maar ook veel schrijfwerk. Handmatige cleanup nodig |
| Creation Functions | [`of()`](#of), [`from()`](#from), [`fromEvent()`](#fromevent), [`interval()`](#interval-timer), [`timer()`](#interval-timer), [`ajax()`](#ajax), [`fromFetch()`](#fromfetch), [`scheduled()`](#scheduled) | Veelgebruikte functies voor data-, event- en tijdgebaseerde aanmaak |
| Speciale Creation Functions | [`defer()`](#defer), [`range()`](#range), [`generate()`](#generate), [`iif()`](#iif) | Controle-achtige, loop-achtige aanmaak, conditionele omschakeling, etc. |
| Speciale Observable | [`EMPTY`](#empty-never-throwerror), [`NEVER`](#empty-never-throwerror), [`throwError()`](#empty-never-throwerror) | Voor voltooiing, niets doen, foutuitgifte |
| Subject familie | [`Subject`](#subject-behaviorsubject), [`BehaviorSubject`](#subject-behaviorsubject) | Speciale Observable die zowel als observer als als zender functioneert |
| Callback conversie | [`bindCallback()`](#bindcallback), [`bindNodeCallback()`](#bindnodecallback) | Converteert callback-gebaseerde functies naar Observable |
| Resource controle | [`using()`](#using) | Voert resource controle uit samen met Observable abonnement |
| WebSocket | [`webSocket()`](#websocket) | Behandelt WebSocket-communicatie als bidirectionele Observable |



## Aangepaste creatie

### new Observable()
[📘 RxJS Officieel: Observable](https://rxjs.dev/api/index/class/Observable)


De meest basale methode is het direct gebruiken van de `Observable` constructor. Deze methode is het meest flexibel wanneer je aangepaste Observable logica wilt definiëren. Fijnmazige gedragscontrole is mogelijk door expliciete `next`, `error`, `complete` aanroepen.

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
  next: value => console.log('Waarde:', value),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Voltooid')
});
// Output:
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Waarde: 4
// Voltooid
```

> [!CAUTION]
> Bij gebruik van `new Observable()` moet je expliciet resource vrijgave (cleanup processing) zelf schrijven.
> ```ts
> const obs$ = new Observable(subscriber => {
>   const id = setInterval(() => subscriber.next(Date.now()), 1000);
>   return () => {
>     clearInterval(id); // Expliciete resource vrijgave
>   };
> });
> ```
> Aan de andere kant hebben RxJS ingebouwde creation functions zoals `fromEvent()` en `interval()` intern passende cleanup processing.
> ```ts
> const click$ = fromEvent(document, 'click');
> const timer$ = interval(1000);
> ```
> Deze gebruiken intern `addEventListener` of `setInterval`, en zijn zo ontworpen dat RxJS automatisch `removeEventListener` of `clearInterval()` aanroept bij `unsubscribe()`.
>
> Merk op dat zelfs als cleanup processing intern in RxJS is geïmplementeerd, die processing niet wordt uitgevoerd als je `unsubscribe()` niet aanroept.
> ```ts
>  const subscription = observable$.subscribe({
>  //weggelaten...
>  });
>
>  subscription.unsubscribe(); // 👈
> ```
> - Maak het een gewoonte om altijd `unsubscribe()` te gebruiken wanneer het niet meer nodig is, ongeacht de Observable-aanmaakmethode.
> - Als je de afmelding vergeet, blijven event listeners en timers actief, wat geheugen leaks en onverwachte bijeffecten kan veroorzaken.


## Creation Functions (aanmaakfuncties)

Voor beknoptere en gebruikspecifieke Observable-aanmaak zijn de door RxJS aangeboden "Creation Functions (aanmaakfuncties)" handig. Voor vaak herhaalde use cases vereenvoudigen deze je code.

> [!NOTE]
> In de officiële RxJS documentatie worden deze geclassificeerd als "Creation Functions (aanmaakfuncties)".
> Vroeger (RxJS 5.x ~ 6) werden ze "creation operator (aanmaak operator)" genoemd, maar vanaf RxJS 7 is "Creation Functions" de officiële term.


### of()
[📘 RxJS Officieel: of()](https://rxjs.dev/api/index/function/of)

De eenvoudigste Observable aanmaakfunctie die meerdere waarden **één voor één in volgorde uitgeeft**.


```ts
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Waarde:', value),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Voltooid')
});
// Output: Waarde: 1, Waarde: 2, Waarde: 3, Waarde: 4, Waarde: 5, Voltooid
```

> [!IMPORTANT]
> Verschil tussen `of()` en `from()`
> - `of([1, 2, 3])` → Geeft één array uit.
> - `from([1, 2, 3])` → Geeft individuele waarden `1`, `2`, `3` achter elkaar uit.
>
> Dit wordt vaak verward, dus wees voorzichtig.

> [!TIP]
> Zie [of() detailpagina](/nl/guide/creation-functions/basic/of) voor gedetailleerd gebruik en praktische voorbeelden.

### from()
[📘 RxJS Officieel: from()](https://rxjs.dev/api/index/function/from)

**Genereert Observable uit bestaande datastructuren** zoals arrays, Promises, iterables.

```ts
import { from } from 'rxjs';

// Aanmaken vanuit array
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Array waarde:', value),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Voltooid')
});

// Aanmaken vanuit Promise
const promise$ = from(Promise.resolve('Promise resultaat'));
promise$.subscribe({
  next: value => console.log('Promise resultaat:', value),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Voltooid')
});

// Aanmaken vanuit iterable
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Iterable waarde:', value),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Voltooid')
});

// Output:
// Array waarde: 1
// Array waarde: 2
// Array waarde: 3
// Voltooid
// Iterable waarde: 1
// Iterable waarde: 2
// Iterable waarde: 3
// Voltooid
// Promise resultaat: Promise resultaat
// Voltooid
```

> [!TIP]
> Zie [from() detailpagina](/nl/guide/creation-functions/basic/from) voor gedetailleerd gebruik en praktische voorbeelden.

### fromEvent()
[📘 RxJS Officieel: fromEvent](https://rxjs.dev/api/index/function/fromEvent)

Functie om **event sources zoals DOM events als Observable te behandelen**.

```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe({
  next: event => console.log('Klik event:', event),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Voltooid')
});

// Output:
// Klik event: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!CAUTION]
> Let op ondersteunde event targets
> - `fromEvent()` ondersteunt browser DOM elementen (EventTarget implementatie), Node.js EventEmitter, jQuery-achtige event targets.
> - Bij meerdere abonnementen kunnen meerdere event listeners worden toegevoegd.

> 👉 Zie [Streamificatie van Events](../observables/events) voor meer gedetailleerde voorbeelden van event stream gebruik.

> [!TIP]
> Zie [fromEvent() detailpagina](/nl/guide/creation-functions/basic/fromEvent) voor gedetailleerd gebruik en praktische voorbeelden.

### interval(), timer()
[📘 RxJS Officieel: interval](https://rxjs.dev/api/index/function/interval), [📘 RxJS Officieel: timer](https://rxjs.dev/api/index/function/timer)

Gebruikt wanneer je continu waarden op regelmatige intervallen wilt uitgeven of **tijdscontrole nodig is**.

```ts
import { interval, timer } from 'rxjs';

// Geeft waarden uit elke seconde
const interval$ = interval(1000);
interval$.subscribe({
  next: value => console.log('Interval:', value),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Voltooid')
});

// Start na 3 seconden, daarna elke seconde waarden uitgeven
const timer$ = timer(3000, 1000);
timer$.subscribe({
  next: value => console.log('Timer:', value),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Voltooid')
});

// Output:
// Interval: 0
// Interval: 1
// Interval: 2
// Timer: 0
// Interval: 3
// Timer: 1
// Interval: 4
// Timer: 2
// .
// .
```
`interval()` en `timer()` worden frequent gebruikt voor tijdsgestuurde processing, met name geschikt voor animaties, polling, asynchrone event vertragingen.

> [!CAUTION]
> Let op dat het Cold Observables zijn
> - `interval()` en `timer()` zijn Cold Observables en worden onafhankelijk uitgevoerd bij elk abonnement.
> - Overweeg indien nodig Hot te maken met `share()` en dergelijke.
>
> Zie de sectie ["Cold Observable en Hot Observable"](./cold-and-hot-observables.md)" voor details.

> [!TIP]
> Zie [interval() detailpagina](/nl/guide/creation-functions/basic/interval) en [timer() detailpagina](/nl/guide/creation-functions/basic/timer) voor gedetailleerd gebruik en praktische voorbeelden.

### ajax()
[📘 RxJS Officieel: ajax](https://rxjs.dev/api/ajax/ajax)

Functie om HTTP-communicatieresultaten **asynchroon als Observable te behandelen**.

```ts
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
api$.subscribe({
  next: response => console.log('API response:', response),
  error: error => console.error('API fout:', error),
  complete: () => console.log('API voltooid')
});

// Output:
// API response: {userId: 1, id: 1, title: 'delectus aut autem', completed: false}
//  API voltooid
```

> [!NOTE]
> RxJS ajax gebruikt intern XMLHttpRequest. Aan de andere kant heeft RxJS ook een fromFetch operator, die Fetch API gebruikt om HTTP requests uit te voeren.

> [!TIP]
> Zie [ajax() detailpagina](/nl/guide/creation-functions/http-communication/ajax) voor gedetailleerd gebruik en praktische voorbeelden. Zie [HTTP Communication Creation Functions](/nl/guide/creation-functions/http-communication/) voor een overzicht van HTTP-communicatiefuncties.

### fromFetch()
[📘 RxJS Officieel: fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` is een functie die Fetch API omhult en HTTP requests als Observable kan behandelen.
Het lijkt op `ajax()`, maar dit is moderner en lichter.

```ts
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs';

const api$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1');

api$.pipe(
  switchMap(response => response.json())
).subscribe({
  next: data => console.log('Data:', data),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Voltooid')
});

// Output:
// Data: Objectcompleted: falseid: 1title: "delectus aut autem"userId: 1[[Prototype]]: Object
// Voltooid
```

> [!NOTE]
> Omdat `fromFetch()` Fetch API gebruikt, moet je, in tegenstelling tot `ajax()`, handmatig request-initialisatie en response `.json()` conversie uitvoeren.
> Je moet ook passend error handling en HTTP status controles uitvoeren.

> [!TIP]
> Zie [fromFetch() detailpagina](/nl/guide/creation-functions/http-communication/fromFetch) voor gedetailleerd gebruik en praktische voorbeelden. Zie [HTTP Communication Creation Functions](/nl/guide/creation-functions/http-communication/) voor een overzicht van HTTP-communicatiefuncties.

### scheduled()
[📘 RxJS Officieel: scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` is een functie waarmee je expliciet een scheduler kunt specificeren bij uitgavefuncties zoals `of()` en `from()`.
Gebruik dit wanneer je synchrone/asynchrone uitvoeringstiming nauwkeurig wilt controleren.

```ts
import { scheduled, asyncScheduler } from 'rxjs';

const observable$ = scheduled([1, 2, 3], asyncScheduler);
observable$.subscribe({
  next: val => console.log('Waarde:', val),
  complete: () => console.log('Voltooid')
});

// Uitvoering gebeurt asynchroon
// Output:
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Voltooid
```

> [!NOTE]
> Door `scheduled()` te gebruiken, kun je bestaande synchrone functies (bijv: `of()`, `from()`) asynchroon laten werken.
> Dit is handig voor testen en UI-prestatie-optimalisatie die asynchrone processing control vereisen.

> [!TIP]
> Zie [scheduled() detailpagina](/nl/guide/creation-functions/control/scheduled) voor gedetailleerd gebruik en praktische voorbeelden. Zie [Control Creation Functions](/nl/guide/creation-functions/control/) voor een overzicht van controlefuncties.

### defer()
[📘 RxJS Officieel: defer](https://rxjs.dev/api/index/function/defer)

Gebruikt wanneer je **het aanmaken van Observable wilt uitstellen tot het moment van abonnement**.

```ts
import { defer, of } from 'rxjs';

const deferred$ = defer(() => {
  const randomValue = Math.random();
  return randomValue > 0.5 ?
    of('Waarde groter dan 50%:', randomValue) :
    of('Waarde 50% of minder:', randomValue);
});

// Bij elk abonnement wordt een nieuwe Observable aangemaakt
deferred$.subscribe(value => console.log(value));
deferred$.subscribe(value => console.log(value));

// Output:
// Waarde 50% of minder:
// 0.08011364416212319
// Waarde 50% of minder:
// 0.3141403962502316
```
`defer()` is effectief wanneer je processing met bijeffecten niet op het moment van Observable-aanmaak, maar op het moment van abonnement wilt uitstellen. Geschikt voor toepassingen zoals random generatie of het ophalen van de huidige tijd.

> [!IMPORTANT]
> Verschil met `of()`
> - Met `of()` wordt de waarde bepaald op het moment van aanmaak.
> - `defer()` wordt voor het eerst verwerkt op het moment van abonnement, dus geschikt voor processing waarbij de waarde bij elk abonnement verandert.

### range()
[📘 RxJS Officieel: range](https://rxjs.dev/api/index/function/range)

Maakt een Observable die een reeks getallen binnen een bepaald bereik uitgeeft.

```ts
import { range } from 'rxjs';

const range$ = range(5, 3); // Vanaf 5, 3 getallen → 5, 6, 7
range$.subscribe({
  next: val => console.log('range:', val),
  complete: () => console.log('Voltooid')
});

// Output:
// range: 5
// range: 6
// range: 7
// Voltooid
```

### generate()
[📘 RxJS Officieel: generate](https://rxjs.dev/api/index/function/generate)

Functie om **getallen of status loop-achtig te genereren** door een beginwaarde, voorwaarde en update-expressie te specificeren.

```ts
import { generate } from 'rxjs';

const generate$ = generate({
  initialState: 0,
  condition: x => x < 5,
  iterate: x => x + 1
});

generate$.subscribe({
  next: val => console.log('generate:', val),
  complete: () => console.log('Voltooid')
});

// Output:
// generate: 0
// generate: 1
// generate: 2
// generate: 3
// generate: 4
// Voltooid
```

### iif()
[📘 RxJS Officieel: iif](https://rxjs.dev/api/index/function/iif)

Functie om **de uit te voeren Observable dynamisch te wisselen** volgens een voorwaarde.

```ts
import { iif, of, EMPTY } from 'rxjs';

const condition = true;
const iif$ = iif(() => condition, of('Voorwaarde is true'), EMPTY);

iif$.subscribe({
  next: val => console.log('iif:', val),
  complete: () => console.log('Voltooid')
});

// Output:
// iif: Voorwaarde is true
// Voltooid
```

> [!NOTE]
> `iif()` kan de terug te geven Observable dynamisch wisselen volgens een voorwaarde. Handig voor flow control.

## Speciale Observable

### EMPTY, NEVER, throwError()
[📘 RxJS Officieel: EMPTY](https://rxjs.dev/api/index/const/EMPTY), [📘 RxJS Officieel: NEVER](https://rxjs.dev/api/index/const/NEVER), [📘 RxJS Officieel: throwError](https://rxjs.dev/api/index/function/throwError)


RxJS biedt ook speciale Observables die nuttig zijn voor uitvoeringscontrole, exception handling en leerdoeleinden.

```ts
import { EMPTY, throwError, NEVER } from 'rxjs';

// Observable die onmiddellijk voltooit
const empty$ = EMPTY;
empty$.subscribe({
  next: () => console.log('Dit wordt niet weergegeven'),
  complete: () => console.log('Onmiddellijk voltooid')
});

// Observable die een fout uitgeeft
const error$ = throwError(() => new Error('Fout opgetreden'));
error$.subscribe({
  next: () => console.log('Dit wordt niet weergegeven'),
  error: err => console.error('Fout:', err.message),
  complete: () => console.log('Voltooid')
});

// Observable die niets uitgeeft en niet voltooit
const never$ = NEVER;
never$.subscribe({
  next: () => console.log('Dit wordt niet weergegeven'),
  complete: () => console.log('Dit wordt ook niet weergegeven')
});

// Output:
// Onmiddellijk voltooid
// Fout: Fout opgetreden
```

> [!IMPORTANT]
> Hoofdzakelijk voor controle-, verificatie- en leerdoeleinden
> - `EMPTY`, `NEVER`, `throwError()` worden niet voor normale datastromen gebruikt, maar voor **flow control en exception handling verificatie**, of voor leerdoeleinden.


## Subject familie

### Subject, BehaviorSubject etc. {#subject-behaviorsubject}
[📘 RxJS Officieel: Subject](https://rxjs.dev/api/index/class/Subject), [📘 RxJS Officieel: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Observable die zelf waarden kan uitgeven, geschikt voor **multicasting en status delen**.

```ts
import { Subject } from 'rxjs';

const subject$ = new Subject<number>();

// Gebruik als Observer
subject$.subscribe(value => console.log('Observer 1:', value));
subject$.subscribe(value => console.log('Observer 2:', value));

// Gebruik als Observable
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
> Let op dat het Hot Observable is
> - `Subject` wordt "gelijktijdig" aan abonnees genotificeerd, dus in tegenstelling tot Cold Observables zoals `from()` en `of()`, **kan het gebeuren dat je waarden niet kunt ontvangen afhankelijk van het abonnementstiming**.

Zie ["Wat is Subject"](../subjects/what-is-subject.md) voor details.


## Callback conversie

RxJS biedt `bindCallback()` en `bindNodeCallback()` als functies om callback-gebaseerde asynchrone functies naar Observable te converteren.

### bindCallback()
[📘 RxJS Officieel: bindCallback](https://rxjs.dev/api/index/function/bindCallback)

`bindCallback()` converteert een asynchrone functie waarbij "het laatste argument een callback functie is" naar een "functie die Observable retourneert".

```ts
import { bindCallback } from 'rxjs';

// Asynchrone functie in callback vorm
function asyncFn(input: string, callback: (result: string) => void) {
  setTimeout(() => callback(`Hallo, ${input}`), 1000);
}

// Converteer asyncFn naar "functie die Observable retourneert"
const observableFn = bindCallback(asyncFn);
const result$ = observableFn('RxJS');

result$.subscribe({
  next: val => console.log(val), // Hallo, RxJS
  complete: () => console.log('Voltooid')
});

// Output:
// Hallo, RxJS
// Voltooid
```

### bindNodeCallback()
[📘 RxJS Officieel: bindNodeCallback](https://rxjs.dev/api/index/function/bindNodeCallback)

`bindNodeCallback()` converteert Node.js "error-first callback (err, result) vorm" functies naar "functies die Observable retourneren".

```ts
import { bindNodeCallback } from 'rxjs';
import { readFile } from 'fs';

// Converteer readFile naar "functie die Observable retourneert" (fout wordt via error genotificeerd)
const readFile$ = bindNodeCallback(readFile);
readFile$('./some.txt').subscribe({
  next: data => console.log('Inhoud:', data),
  error: err => console.error('Fout:', err)
});
```

> [!NOTE]
> `bindNodeCallback()` is compatibel met Node.js `(err, result)` type asynchrone functies.

### Verschil tussen bindCallback() en bindNodeCallback()
Het verschil tussen bindCallback() en bindNodeCallback() is het formaat van de doelcallback functie.

|Functie|Doelfunctie formaat|Kenmerk|
|---|---|---|
|bindCallback()|callback(result)|Compatibel met normale callback (1 argument)|
|bindNodeCallback()|callback(error, result) |Compatibel met Node.js-stijl error-first formaat|

#### Concreet voorbeeld: doel van bindCallback()

```ts
function doSomething(input: string, callback: (result: string) => void) {
  callback(`Resultaat: ${input}`);
}
```
→ Kan worden geconverteerd met bindCallback()


#### Concreet voorbeeld: doel van bindNodeCallback() (Node.js-stijl)

```ts
function readFile(path: string, cb: (err: Error | null, data: string) => void) {
  if (path === 'valid.txt') cb(null, 'file content');
  else cb(new Error('not found'), '');
}
```
→ Met bindNodeCallback() wordt error bij fout als Observable genotificeerd.


> [!NOTE]
> Hoe te kiezen
> - Als het eerste argument van de callback "error of niet" is → bindNodeCallback()
> - Als het simpelweg "alleen een waarde retourneert" callback is → bindCallback()

## Resource controle

### using()
[📘 RxJS Officieel: using](https://rxjs.dev/api/index/function/using)

`using()` is een functie om resource-aanmaak en vrijgave te koppelen aan de lifecycle van Observable.
Handig in combinatie met **handmatige cleanup die nodig is** zoals WebSocket, event listeners, externe resources.

```ts
import { using, interval, Subscription } from 'rxjs';

const resource$ = using(
  () => new Subscription(() => console.log('Resource vrijgave')),
  () => interval(1000)
);

const sub = resource$.subscribe(value => console.log('Waarde:', value));

// Afmelden na enkele seconden
setTimeout(() => sub.unsubscribe(), 3500);

// Output:
// Waarde: 0
// Waarde: 1
// Waarde: 2
// Resource vrijgave
```

> [!IMPORTANT]
> `using()` is handig om de scope van resources te synchroniseren met Observable abonnement.
> Op het moment van `unsubscribe()` wordt expliciete cleanup processing automatisch aangeroepen.

> [!TIP]
> Zie [using() detailpagina](/nl/guide/creation-functions/control/using) voor gedetailleerd gebruik en praktische voorbeelden. Zie [Control Creation Functions](/nl/guide/creation-functions/control/) voor een overzicht van controlefuncties.

## WebSocket()
[📘 RxJS Officieel: webSocket](https://rxjs.dev/api/webSocket/webSocket)

De `rxjs/webSocket` module van RxJS heeft een `webSocket()` functie waarmee WebSocket als Observable/Observer kan worden behandeld.

```ts
import { webSocket } from 'rxjs/webSocket';

const socket$ = webSocket('wss://echo.websocket.org');

socket$.subscribe({
  next: msg => console.log('Ontvangen:', msg),
  error: err => console.error('Fout:', err),
  complete: () => console.log('Voltooid')
});

// Bericht verzenden (functionaliteit als Observer)
socket$.next('Hallo WebSocket!');
```

> [!IMPORTANT]
> `webSocket()` is een Observable/Observer hybride die bidirectionele communicatie mogelijk maakt.
> Omdat WebSocket-verbinding, verzending en ontvangst eenvoudig als Observable kunnen worden behandeld, is het handig voor realtime communicatie.


## Samenvatting

RxJS streams maken het mogelijk om traditionele JavaScript event handling en AJAX-communicatie via een uniforme interface te behandelen. Het toont zijn kracht vooral bij het werken met data die in de tijd verandert, of bij het combineren van meerdere event sources.
