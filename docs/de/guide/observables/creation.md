---
description: "Systematische Erklärung der Methoden zur Erstellung von Observables in RxJS, von grundlegenden Erstellungsfunktionen wie of und from bis zur Definition benutzerdefinierter Observables, der Umwandlung von HTTP-Kommunikation in Streams und der Konvertierung von Events zu Observables. Einführung in Implementierungsmuster unter Nutzung der Typinferenz von TypeScript."
---
# Methoden zur Erstellung von Observables

Ein Observable ist die Definition eines "Datenstroms", und es gibt viele Möglichkeiten, es zu erstellen.
RxJS bietet verschiedene Methoden, um benutzerdefinierte Observables zu erstellen oder Observables einfach aus Events, Arrays, HTTP-Antworten usw. zu generieren.

Hier stellen wir die Methoden zur Erstellung von Observables in RxJS umfassend vor, von grundlegender Syntax bis zu praktischen Anwendungsfällen.

## Klassifizierung der Observable-Erstellungsmethoden

Nachfolgend eine Übersicht der wichtigsten Erstellungsmethoden nach Kategorien.

| Kategorie | Hauptmethoden | Beschreibung |
|----------|----------|------|
| Benutzerdefinierte Erstellung | [`new Observable()`](#new-observable) | Hohe Flexibilität, aber auch mehr Code. Manuelle Bereinigung erforderlich |
| Creation Functions | [`of()`](#of), [`from()`](#from), [`fromEvent()`](#fromevent), [`interval()`](#interval-timer), [`timer()`](#interval-timer), [`ajax()`](#ajax), [`fromFetch()`](#fromfetch), [`scheduled()`](#scheduled) | Häufig verwendete Erstellungsfunktionen für Daten, Events und zeitbasierte Funktionen |
| Spezielle Creation Functions | [`defer()`](#defer), [`range()`](#range), [`generate()`](#generate), [`iif()`](#iif) | Kontroll- und schleifenbasierte Erstellung, bedingte Umschaltung usw. |
| Spezielle Observables | [`EMPTY`](#empty-never-throwerror), [`NEVER`](#empty-never-throwerror), [`throwError()`](#empty-never-throwerror) | Für Abschluss, Nichtstun, Fehlerausgabe |
| Subject-Typen | [`Subject`](#subject-behaviorsubject), [`BehaviorSubject`](#subject-behaviorsubject) | Spezielle Observables, die sowohl als Beobachter als auch als Sender fungieren |
| Callback-Konvertierung | [`bindCallback()`](#bindcallback), [`bindNodeCallback()`](#bindnodecallback) | Konvertierung von Callback-basierten Funktionen zu Observables |
| Ressourcenkontrolle | [`using()`](#using) | Ressourcenkontrolle gleichzeitig mit Observable-Subscription |
| WebSocket | [`webSocket()`](#websocket) | Behandlung von WebSocket-Kommunikation als bidirektionales Observable |



## Benutzerdefinierte Erstellung

### new Observable()
[📘 RxJS Official: Observable](https://rxjs.dev/api/index/class/Observable)


Die grundlegendste Methode ist die direkte Verwendung des `Observable`-Konstruktors. Diese Methode ist am flexibelsten, wenn Sie benutzerdefinierte Observable-Logik definieren möchten. Durch explizite `next`-, `error`- und `complete`-Aufrufe ist eine feine Verhaltenssteuerung möglich.

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
  next: value => console.log('Wert:', value),
  error: err => console.error('Fehler:', err),
  complete: () => console.log('Abgeschlossen')
});
// Ausgabe:
// Wert: 1
// Wert: 2
// Wert: 3
// Wert: 4
// Abgeschlossen
```

> [!CAUTION]
> Wenn Sie `new Observable()` verwenden, müssen Sie die explizite Ressourcenfreigabe (Bereinigungsverarbeitung) selbst schreiben.
> ```ts
> const obs$ = new Observable(subscriber => {
>   const id = setInterval(() => subscriber.next(Date.now()), 1000);
>   return () => {
>     clearInterval(id); // Explizite Ressourcenfreigabe
>   };
> });
> ```
> Andererseits haben RxJS-Built-in-Erstellungsfunktionen wie `fromEvent()` oder `interval()` intern eine geeignete Bereinigungsverarbeitung.
> ```ts
> const click$ = fromEvent(document, 'click');
> const timer$ = interval(1000);
> ```
> Diese verwenden intern `addEventListener` bzw. `setInterval` und sind so konzipiert, dass RxJS beim `unsubscribe()` automatisch `removeEventListener` bzw. `clearInterval()` aufruft.
>
> Beachten Sie, dass auch wenn die Bereinigungsverarbeitung intern in RxJS implementiert ist, diese Verarbeitung nicht ausgeführt wird, wenn `unsubscribe()` nicht aufgerufen wird.
> ```ts
>  const subscription = observable$.subscribe({
>  // ausgelassen...
>  });
>
>  subscription.unsubscribe(); // 👈
> ```
> - Gewöhnen Sie sich an, immer `unsubscribe()` aufzurufen, wenn Sie ein Observable nicht mehr benötigen, unabhängig von der Erstellungsmethode.
> - Wenn Sie die Subscription-Aufhebung vergessen, laufen Event-Listener oder Timer weiter, was zu Speicherlecks oder unerwarteten Nebeneffekten führen kann.


## Creation Functions (Erstellungsfunktionen)

Für eine prägnantere und zweckorientiertere Observable-Erstellung sind die von RxJS bereitgestellten "Creation Functions" praktisch. Für wiederkehrende Anwendungsfälle vereinfacht deren Verwendung den Code erheblich.

> [!NOTE]
> In der offiziellen RxJS-Dokumentation werden diese als "Creation Functions" klassifiziert.
> Früher (RxJS 5.x ~ 6) wurden sie als "creation operator" bezeichnet, aber ab RxJS 7 ist "Creation Functions" der offizielle Begriff.


### of()
[📘 RxJS Official: of()](https://rxjs.dev/api/index/function/of)

Die einfachste Observable-Erstellungsfunktion, die mehrere Werte **nacheinander einzeln ausgibt**.


```ts
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Wert:', value),
  error: err => console.error('Fehler:', err),
  complete: () => console.log('Abgeschlossen')
});
// Ausgabe: Wert: 1, Wert: 2, Wert: 3, Wert: 4, Wert: 5, Abgeschlossen
```

> [!IMPORTANT]
> Unterschied zwischen `of()` und `from()`
> - `of([1, 2, 3])` → Gibt ein einzelnes Array aus.
> - `from([1, 2, 3])` → Gibt einzelne Werte `1`, `2`, `3` nacheinander aus.
>
> Diese werden oft verwechselt, daher ist Vorsicht geboten.

> [!TIP]
> Für detaillierte Verwendung und praktische Beispiele siehe [of() Detailseite](/de/guide/creation-functions/basic/of).

### from()
[📘 RxJS Official: from()](https://rxjs.dev/api/index/function/from)

**Generiert Observables aus bestehenden Datenstrukturen** wie Arrays, Promises, Iterables usw.

```ts
import { from } from 'rxjs';

// Aus Array erstellen
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Array-Wert:', value),
  error: err => console.error('Fehler:', err),
  complete: () => console.log('Abgeschlossen')
});

// Aus Promise erstellen
const promise$ = from(Promise.resolve('Promise-Ergebnis'));
promise$.subscribe({
  next: value => console.log('Promise-Ergebnis:', value),
  error: err => console.error('Fehler:', err),
  complete: () => console.log('Abgeschlossen')
});

// Aus Iterable erstellen
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Iterable-Wert:', value),
  error: err => console.error('Fehler:', err),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Array-Wert: 1
// Array-Wert: 2
// Array-Wert: 3
// Abgeschlossen
// Iterable-Wert: 1
// Iterable-Wert: 2
// Iterable-Wert: 3
// Abgeschlossen
// Promise-Ergebnis: Promise-Ergebnis
// Abgeschlossen
```

> [!TIP]
> Für detaillierte Verwendung und praktische Beispiele siehe [from() Detailseite](/de/guide/creation-functions/basic/from).

### fromEvent()
[📘 RxJS Official: fromEvent](https://rxjs.dev/api/index/function/fromEvent)

Eine Funktion zur **Behandlung von Event-Quellen wie DOM-Events als Observable**.

```ts
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
clicks$.subscribe({
  next: event => console.log('Click-Event:', event),
  error: err => console.error('Fehler:', err),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Click-Event: PointerEvent {isTrusted: true, pointerId: 1, width: 1, height: 1, pressure: 0, …}
```

> [!CAUTION]
> Beachten Sie unterstützte Event-Ziele
> - `fromEvent()` unterstützt Browser-DOM-Elemente (EventTarget-Implementierung), Node.js EventEmitter und jQuery-ähnliche Event-Ziele.
> - Bei mehrfachen Subscriptions können mehrere Event-Listener hinzugefügt werden.

> 👉 Für detailliertere Beispiele zur Verwendung von Event-Streams siehe [Event-Umwandlung in Streams](../observables/events).

> [!TIP]
> Für detaillierte Verwendung und praktische Beispiele siehe [fromEvent() Detailseite](/de/guide/creation-functions/basic/fromEvent).

### interval(), timer()
[📘 RxJS Official: interval](https://rxjs.dev/api/index/function/interval), [📘 RxJS Official: timer](https://rxjs.dev/api/index/function/timer)

Werden verwendet, wenn Sie Werte in regelmäßigen Abständen kontinuierlich ausgeben möchten oder **Zeitsteuerung benötigen**.

```ts
import { interval, timer } from 'rxjs';

// Wert alle 1 Sekunde ausgeben
const interval$ = interval(1000);
interval$.subscribe({
  next: value => console.log('Intervall:', value),
  error: err => console.error('Fehler:', err),
  complete: () => console.log('Abgeschlossen')
});

// Nach 3 Sekunden starten, danach alle 1 Sekunde Wert ausgeben
const timer$ = timer(3000, 1000);
timer$.subscribe({
  next: value => console.log('Timer:', value),
  error: err => console.error('Fehler:', err),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Intervall: 0
// Intervall: 1
// Intervall: 2
// Timer: 0
// Intervall: 3
// Timer: 1
// Intervall: 4
// Timer: 2
// .
// .
```
`interval()` und `timer()` werden häufig für zeitbasierte Verarbeitung verwendet, besonders geeignet für Animationen, Polling, asynchrone Event-Verzögerungen usw.

> [!CAUTION]
> Beachten Sie, dass es Cold Observables sind
> - `interval()` und `timer()` sind Cold Observables und werden bei jeder Subscription unabhängig ausgeführt.
> - Bei Bedarf können Sie sie mit `share()` in Hot umwandeln.
>
> Siehe Abschnitt "[Cold Observable und Hot Observable](./cold-and-hot-observables.md)" für Details.

> [!TIP]
> Für detaillierte Verwendung und praktische Beispiele siehe [interval() Detailseite](/de/guide/creation-functions/basic/interval) und [timer() Detailseite](/de/guide/creation-functions/basic/timer).

### ajax()
[📘 RxJS Official: ajax](https://rxjs.dev/api/ajax/ajax)

Eine Funktion zur **asynchronen Behandlung von HTTP-Kommunikationsergebnissen als Observable**.

```ts
import { ajax } from 'rxjs/ajax';

const api$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1');
api$.subscribe({
  next: response => console.log('API-Antwort:', response),
  error: error => console.error('API-Fehler:', error),
  complete: () => console.log('API abgeschlossen')
});

// Ausgabe:
// API-Antwort: {userId: 1, id: 1, title: 'delectus aut autem', completed: false}
// API abgeschlossen
```

> [!NOTE]
> RxJS ajax verwendet intern XMLHttpRequest. Andererseits gibt es in RxJS auch einen fromFetch-Operator, der die Fetch API für HTTP-Anfragen verwendet.

> [!TIP]
> Für detaillierte Verwendung und praktische Beispiele siehe [ajax() Detailseite](/de/guide/creation-functions/http-communication/ajax). Für Übersicht über HTTP-Kommunikation siehe [HTTP-Kommunikations-Creation Functions](/de/guide/creation-functions/http-communication/).

### fromFetch()
[📘 RxJS Official: fromFetch](https://rxjs.dev/api/fetch/fromFetch)

`fromFetch()` ist eine Funktion, die die Fetch API umschließt und HTTP-Anfragen als Observable behandeln kann.
Ähnlich wie `ajax()`, aber moderner und leichtgewichtiger.

```ts
import { fromFetch } from 'rxjs/fetch';
import { switchMap } from 'rxjs';

const api$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1');

api$.pipe(
  switchMap(response => response.json())
).subscribe({
  next: data => console.log('Daten:', data),
  error: err => console.error('Fehler:', err),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Daten: {completed: false, id: 1, title: "delectus aut autem", userId: 1}
// Abgeschlossen
```

> [!NOTE]
> Da `fromFetch()` die Fetch API verwendet, müssen Sie im Gegensatz zu `ajax()` die Anfragekonfiguration und die `.json()`-Konvertierung der Antwort manuell durchführen.
> Auch Fehlerbehandlung und HTTP-Statusprüfungen müssen ordnungsgemäß durchgeführt werden.

> [!TIP]
> Für detaillierte Verwendung und praktische Beispiele siehe [fromFetch() Detailseite](/de/guide/creation-functions/http-communication/fromFetch). Für Übersicht über HTTP-Kommunikation siehe [HTTP-Kommunikations-Creation Functions](/de/guide/creation-functions/http-communication/).

### scheduled()
[📘 RxJS Official: scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` ist eine Funktion, die es ermöglicht, Ausgabefunktionen wie `of()` oder `from()` explizit mit einem Scheduler zu versehen.
Wird verwendet, wenn Sie das synchrone/asynchrone Ausführungs-Timing genau steuern möchten.

```ts
import { scheduled, asyncScheduler } from 'rxjs';

const observable$ = scheduled([1, 2, 3], asyncScheduler);
observable$.subscribe({
  next: val => console.log('Wert:', val),
  complete: () => console.log('Abgeschlossen')
});

// Ausführung erfolgt asynchron
// Ausgabe:
// Wert: 1
// Wert: 2
// Wert: 3
// Abgeschlossen
```

> [!NOTE]
> Durch die Verwendung von `scheduled()` können Sie vorhandene synchrone Funktionen (z.B. `of()`, `from()`) asynchron ausführen lassen.
> Nützlich für Tests und UI-Performance-Optimierung, die asynchrone Verarbeitungssteuerung erfordern.

> [!TIP]
> Für detaillierte Verwendung und praktische Beispiele siehe [scheduled() Detailseite](/de/guide/creation-functions/control/scheduled). Für Übersicht über Steuerung siehe [Steuerungs-Creation Functions](/de/guide/creation-functions/control/).

### defer()
[📘 RxJS Official: defer](https://rxjs.dev/api/index/function/defer)

Wird verwendet, wenn Sie die **Erstellung des Observables bis zur Subscription verzögern möchten**.

```ts
import { defer, of } from 'rxjs';

const deferred$ = defer(() => {
  const randomValue = Math.random();
  return randomValue > 0.5 ?
    of('Wert größer als 50%:', randomValue) :
    of('Wert 50% oder kleiner:', randomValue);
});

// Bei jeder Subscription wird ein neues Observable erstellt
deferred$.subscribe(value => console.log(value));
deferred$.subscribe(value => console.log(value));

// Ausgabe:
// Wert 50% oder kleiner:
// 0.08011364416212319
// Wert 50% oder kleiner:
// 0.3141403962502316
```
`defer()` ist effektiv, wenn Sie Verarbeitung mit Nebeneffekten nicht bei der Observable-Erstellung, sondern bei der Subscription verzögern möchten. Geeignet für Zufallsgenerierung oder Abrufen der aktuellen Zeit.

> [!IMPORTANT]
> Unterschied zu `of()`
> - `of()` legt den Wert zum Erstellungszeitpunkt fest.
> - `defer()` wird erst bei der Subscription verarbeitet, daher geeignet für Verarbeitung, bei der sich der Wert bei jeder Subscription ändert.

### range()
[📘 RxJS Official: range](https://rxjs.dev/api/index/function/range)

Erstellt ein Observable, das eine Reihe von Zahlen in einem bestimmten Bereich ausgibt.

```ts
import { range } from 'rxjs';

const range$ = range(5, 3); // Ab 5, 3 Stück → 5, 6, 7
range$.subscribe({
  next: val => console.log('range:', val),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// range: 5
// range: 6
// range: 7
// Abgeschlossen
```

### generate()
[📘 RxJS Official: generate](https://rxjs.dev/api/index/function/generate)

Eine Funktion zum **schleifenartigen Generieren von Zahlen oder Zuständen** durch Angabe von Anfangswert, Bedingung und Aktualisierungsausdruck.

```ts
import { generate } from 'rxjs';

const generate$ = generate({
  initialState: 0,
  condition: x => x < 5,
  iterate: x => x + 1
});

generate$.subscribe({
  next: val => console.log('generate:', val),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// generate: 0
// generate: 1
// generate: 2
// generate: 3
// generate: 4
// Abgeschlossen
```

### iif()
[📘 RxJS Official: iif](https://rxjs.dev/api/index/function/iif)

Eine Funktion zum **dynamischen Umschalten des auszuführenden Observables** je nach Bedingung.

```ts
import { iif, of, EMPTY } from 'rxjs';

const condition = true;
const iif$ = iif(() => condition, of('Bedingung ist true'), EMPTY);

iif$.subscribe({
  next: val => console.log('iif:', val),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// iif: Bedingung ist true
// Abgeschlossen
```

> [!NOTE]
> `iif()` kann das zurückgegebene Observable je nach Bedingung dynamisch umschalten. Praktisch für Flusssteuerung.

## Spezielle Observables

### EMPTY, NEVER, throwError()
[📘 RxJS Official: EMPTY](https://rxjs.dev/api/index/const/EMPTY), [📘 RxJS Official: NEVER](https://rxjs.dev/api/index/const/NEVER), [📘 RxJS Official: throwError](https://rxjs.dev/api/index/function/throwError)


RxJS bietet auch spezielle Observables, die für Ausführungssteuerung, Ausnahmebehandlung und Lernzwecke nützlich sind.

```ts
import { EMPTY, throwError, NEVER } from 'rxjs';

// Sofort abgeschlossenes Observable
const empty$ = EMPTY;
empty$.subscribe({
  next: () => console.log('Wird nicht angezeigt'),
  complete: () => console.log('Sofort abgeschlossen')
});

// Fehler ausgebendes Observable
const error$ = throwError(() => new Error('Fehler aufgetreten'));
error$.subscribe({
  next: () => console.log('Wird nicht angezeigt'),
  error: err => console.error('Fehler:', err.message),
  complete: () => console.log('Abgeschlossen')
});

// Observable, das nichts ausgibt und auch nicht abschließt
const never$ = NEVER;
never$.subscribe({
  next: () => console.log('Wird nicht angezeigt'),
  complete: () => console.log('Wird auch nicht angezeigt')
});

// Ausgabe:
// Sofort abgeschlossen
// Fehler: Fehler aufgetreten
```

> [!IMPORTANT]
> Hauptsächlich für Steuerungs-, Validierungs- und Lernzwecke
> - `EMPTY`, `NEVER`, `throwError()` werden nicht für normale Datenströme verwendet, sondern für **Flusssteuerung, Validierung der Ausnahmebehandlung** oder Lernzwecke.


## Subject-Typen

### Subject, BehaviorSubject usw. {#subject-behaviorsubject}
[📘 RxJS Official: Subject](https://rxjs.dev/api/index/class/Subject), [📘 RxJS Official: BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Observables, die selbst Werte ausgeben können, geeignet für **Multicasting und Zustandsfreigabe**.

```ts
import { Subject } from 'rxjs';

const subject$ = new Subject<number>();

// Als Observer verwenden
subject$.subscribe(value => console.log('Observer 1:', value));
subject$.subscribe(value => console.log('Observer 2:', value));

// Als Observable verwenden
subject$.next(1);
subject$.next(2);
subject$.next(3);
subject$.complete();

// Ausgabe:
// Observer 1: 1
// Observer 2: 1
// Observer 1: 2
// Observer 2: 2
// Observer 1: 3
// Observer 2: 3
```

> [!IMPORTANT]
> Beachten Sie, dass es Hot Observables sind
> - `Subject` benachrichtigt Subscriber "gleichzeitig", daher können Sie im Gegensatz zu Cold Observables wie `from()` oder `of()` **je nach Subscription-Zeitpunkt Werte verpassen**.

Siehe ["Was ist Subject"](../subjects/what-is-subject.md) für Details.


## Callback-Konvertierung

RxJS bietet `bindCallback()` und `bindNodeCallback()` als Funktionen zur Konvertierung von Callback-basierten asynchronen Funktionen zu Observables.

### bindCallback()
[📘 RxJS Official: bindCallback](https://rxjs.dev/api/index/function/bindCallback)

`bindCallback()` konvertiert eine asynchrone Funktion, deren "letztes Argument eine Callback-Funktion ist", in eine "Funktion, die ein Observable zurückgibt".

```ts
import { bindCallback } from 'rxjs';

// Asynchrone Funktion im Callback-Format
function asyncFn(input: string, callback: (result: string) => void) {
  setTimeout(() => callback(`Hallo, ${input}`), 1000);
}

// asyncFn in "Funktion, die Observable zurückgibt" konvertieren
const observableFn = bindCallback(asyncFn);
const result$ = observableFn('RxJS');

result$.subscribe({
  next: val => console.log(val), // Hallo, RxJS
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Hallo, RxJS
// Abgeschlossen
```

### bindNodeCallback()
[📘 RxJS Official: bindNodeCallback](https://rxjs.dev/api/index/function/bindNodeCallback)

`bindNodeCallback()` konvertiert eine Funktion im "Error-First-Callback (err, result)-Format" von Node.js in eine "Funktion, die ein Observable zurückgibt".

```ts
import { bindNodeCallback } from 'rxjs';
import { readFile } from 'fs';

// readFile in "Funktion, die Observable zurückgibt" konvertieren (Fehler wird über error benachrichtigt)
const readFile$ = bindNodeCallback(readFile);
readFile$('./some.txt').subscribe({
  next: data => console.log('Inhalt:', data),
  error: err => console.error('Fehler:', err)
});
```

> [!NOTE]
> `bindNodeCallback()` unterstützt asynchrone Funktionen im `(err, result)`-Typ von Node.js.

### Unterschied zwischen bindCallback() und bindNodeCallback()
Der Unterschied zwischen bindCallback() und bindNodeCallback() liegt im Format der Ziel-Callback-Funktion.

|Funktion|Ziel-Funktionsformat|Merkmal|
|---|---|---|
|bindCallback()|callback(result)|Unterstützt normale Callbacks (1 Argument)|
|bindNodeCallback()|callback(error, result)|Unterstützt Node.js-Stil Error-First-Format|

#### Konkretes Beispiel: Ziel von bindCallback()

```ts
function doSomething(input: string, callback: (result: string) => void) {
  callback(`Ergebnis: ${input}`);
}
```
→ Kann mit bindCallback() konvertiert werden


#### Konkretes Beispiel: Ziel von bindNodeCallback() (Node.js-Stil)

```ts
function readFile(path: string, cb: (err: Error | null, data: string) => void) {
  if (path === 'valid.txt') cb(null, 'file content');
  else cb(new Error('not found'), '');
}
```
→ Mit bindNodeCallback() wird error bei Fehlerauftreten als Observable benachrichtigt.


> [!NOTE]
> Verwendungsunterscheidung
> - Wenn das erste Callback-Argument "Fehler oder nicht" ist, verwenden Sie bindNodeCallback()
> - Wenn der Callback einfach "nur Wert zurückgibt", verwenden Sie bindCallback()

## Ressourcenkontrolle

### using()
[📘 RxJS Official: using](https://rxjs.dev/api/index/function/using)

`using()` ist eine Funktion zur Verknüpfung von Ressourcenerstellung und -freigabe mit dem Lebenszyklus des Observables.
Praktisch in Kombination mit **manueller Bereinigung erforderlicher Verarbeitung** wie WebSocket, Event-Listener, externen Ressourcen usw.

```ts
import { using, interval, Subscription } from 'rxjs';

const resource$ = using(
  () => new Subscription(() => console.log('Ressourcenfreigabe')),
  () => interval(1000)
);

const sub = resource$.subscribe(value => console.log('Wert:', value));

// Nach einigen Sekunden Subscription aufheben
setTimeout(() => sub.unsubscribe(), 3500);

// Ausgabe:
// Wert: 0
// Wert: 1
// Wert: 2
// Ressourcenfreigabe
```

> [!IMPORTANT]
> `using()` ist praktisch, wenn Sie den Ressourcen-Scope mit der Observable-Subscription in Einklang bringen möchten.
> Zum Zeitpunkt von `unsubscribe()` wird die explizite Bereinigungsverarbeitung automatisch aufgerufen.

> [!TIP]
> Für detaillierte Verwendung und praktische Beispiele siehe [using() Detailseite](/de/guide/creation-functions/control/using). Für Übersicht über Steuerung siehe [Steuerungs-Creation Functions](/de/guide/creation-functions/control/).

## WebSocket()
[📘 RxJS Official: webSocket](https://rxjs.dev/api/webSocket/webSocket)

Das `rxjs/webSocket`-Modul von RxJS bietet die Funktion `webSocket()`, mit der Sie WebSocket als Observable/Observer behandeln können.

```ts
import { webSocket } from 'rxjs/webSocket';

const socket$ = webSocket('wss://echo.websocket.org');

socket$.subscribe({
  next: msg => console.log('Empfangen:', msg),
  error: err => console.error('Fehler:', err),
  complete: () => console.log('Abgeschlossen')
});

// Nachricht senden (Observer-Funktionalität)
socket$.next('Hallo WebSocket!');
```

> [!IMPORTANT]
> `webSocket()` ist ein Observable/Observer-Hybrid mit bidirektionaler Kommunikation.
> Da WebSocket-Verbindung, Senden und Empfangen einfach als Observable behandelt werden können, praktisch für Echtzeitkommunikation.


## Zusammenfassung

RxJS-Streams ermöglichen es, traditionelle JavaScript-Event-Verarbeitung und AJAX-Kommunikation mit einer einheitlichen Schnittstelle zu behandeln. Besonders bei der Verarbeitung zeitlich veränderlicher Daten oder bei der Kombination mehrerer Event-Quellen entfaltet sich die Stärke.
