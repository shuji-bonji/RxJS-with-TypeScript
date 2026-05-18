---
description: "fromEvent() - Creation Function zur Konvertierung von DOM-Ereignissen und EventEmittern in Observable, die Grundlage der ereignisgesteuerten Programmierung, mit der verschiedene Ereignisse wie Klicks, Tastendrucke, Mausbewegungen und Scrollen verarbeitet werden können. Erläutert TypeScript-Typdefinitionen und die Angabe von Ereigniszielen."
---

# fromEvent() - Ereignis in Observable umwandeln

fromEvent()` ist eine Creation Function, die Ereignisquellen wie DOM-Ereignisse und Node.js EventEmitter in Observable-Streams umwandelt.

## Überblick.

fromEvent()` ermöglicht eine ereignisbasierte asynchrone Verarbeitung in der RxJS Pipeline. Es registriert automatisch Event-Listener, wenn sie abonniert werden, und entfernt automatisch Listener, wenn sie abbestellt werden, wodurch das Risiko von Speicherlecks deutlich reduziert wird.

**Signatur**:.

```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

**Offizielle Dokumentation**: [📘 RxJS formula: fromEvent()](https://rxjs.dev/api/index/function/fromEvent)

## Grundlegende Verwendung.

Dies ist das einfachste Beispiel für die Behandlung von DOM-Ereignissen als Observable.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Schaltfläche angeklickt.:', event);
});

// Jedes Mal, wenn sie angeklickt wird, wird ein Ereignis ausgelöst.
```

## Wichtige Merkmale.

### 1. Automatische Registrierung und Deregistrierung von Hörern

Die Funktion `fromEvent()` registriert Ereignis-Listener, wenn sie abonniert sind, und entfernt automatisch Listener, wenn sie abgemeldet sind.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent<MouseEvent>(document, 'click');

const subscription = clicks$.subscribe(event => {
  console.log('Klick-Position:', event.clientX, event.clientY);
});

// 5Abbestellt nach Sekunden (Ereignis-Listener wird auch automatisch gelöscht)
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Abbestellt');
}, 5000);
```

```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

> **Verhinderung von Speicherlecks**
>
> Wenn Sie `unsubscribe()` aufrufen, wird automatisch ein internes `removeEventListener()` ausgeführt. Dadurch entfällt die Notwendigkeit, Hörer manuell zu entfernen, und das Risiko von Speicherlecks wird erheblich reduziert.

### 2) Cold Observable (jedes Abonnement registriert einen unabhängigen Listener).

Das durch fromEvent() erzeugte Observable ist ein **Cold Observable. Jedes Abonnement registriert einen unabhängigen Ereignis-Listener.


```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Aboniert an1 - ListenerAAbonnieren bei
clicks$.subscribe(() => console.log('Observer 1: Klicken Sie auf'));

// 1Abonnieren in Sekunden2hinzufügen - ListenerBRegistrieren unabhängig von
setTimeout(() => {
  clicks$.subscribe(() => console.log('Observer 2: Klicken Sie auf'));
}, 1000);

// 1Beide Listener werden mit einem Klick ausgelöst
// Dies ist ein Beweis dafür, dass jedes Abonnement unabhängige Hörer hat
```

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Schaltfläche angeklickt.:', event);
});

// Jedes Mal, wenn sie angeklickt wird, wird ein Ereignis ausgelöst.
```

> **Proof of Cold Observable
>
> Ein neuer Ereignis-Listener wird jedes Mal registriert, wenn ein Abonnement erstellt wird, und entfernt, wenn das Abonnement abbestellt wird. Dies ist eine Eigenschaft von Cold Observable. Es hat jedoch auch die Hot-Eigenschaft, dass "Ereignisse nicht vor der Subskription empfangen werden können", da die Ereignisquelle (z. B. DOM-Elemente) extern und gemeinsam genutzt wird.

### 3) TypeScript-Typ-Unterstützung

Ereignistypen können explizit angegeben werden.


```typescript
import { fromEvent } from 'rxjs';

const input = document.createElement('input');
input.type = 'text';
document.body.appendChild(input);
const input$ = fromEvent<InputEvent>(input, 'input');

input$.subscribe(event => {
  // eventDer Typ desInputEvent
  const target = event.target as HTMLInputElement;
  console.log('Eingabewertes:', target.value);
});
```

### 4. Cold Observable.

fromEvent()` ist ein **Cold Observable. Jedes Abonnement initiiert eine unabhängige Ausführung.

{```typescript
import { fromEvent } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Aboniert an";
document.body.appendChild(button);

const clicks$ = fromEvent(document, 'click');

// 1Das zweite Abonnement - Ereignis-Listener werden hinzugefügt
clicks$.subscribe(() => console.log('AbonnementA'));

// 2Das zweite Abonnement - Ein weiterer Ereignis-Listener wird hinzugefügt
clicks$.subscribe(() => console.log('AbonnementB'));

// 1Ein Klick löst beide Listener aus
// Ausgabe:
// AbonnementA
// AbonnementB
```

```

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Schaltfläche angeklickt.:', event);
});

// Jedes Mal, wenn sie angeklickt wird, wird ein Ereignis ausgelöst.
```

> **Cold Observable Features**
> - Jedes Abonnement startet eine unabhängige Ausführung
> - Jeder Abonnent erhält seinen eigenen Datenstrom

>
> Weitere Informationen finden Sie unter [Cold Observable und Hot Observable](/de/guide/observables/cold-and-hot-observables).

## Praktische Anwendungsfälle.

### 1. Behandlung von Klick-Ereignissen

Kontrollieren Sie Schaltflächenklicks und verhindern Sie aufeinanderfolgende Klicks.


```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const button = document.createElement('button');
button.innerText = "submit";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  debounceTime(300), // 300msIgnoriere aufeinanderfolgende Klicks innerhalb von
  map(() => 'während der Übertragung...')
).subscribe(message => {
  console.log(message);
  // APIAnrufe und andere Prozesse
});
```

### 2. Echtzeit-Validierung von Formulareingaben

Streamen Sie Eingabeereignisse und führen Sie die Validierung in Echtzeit durch.

```typescript
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'email: ';
const emailInput = document.createElement('input');
label.appendChild(emailInput);
document.body.appendChild(label);
const email$ = fromEvent<InputEvent>(emailInput, 'input');

email$.pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(500), // nachdem die Eingabe gestoppt wurde.500msWird später verarbeitet
  distinctUntilChanged() // Nur wenn sich der Wert ändert
).subscribe(email => {
  console.log('Unterliegt der Validierung:', email);
  // Validierungsprozess für E-Mail-Adressen
  validateEmail(email);
});

function validateEmail(email: string): void {
  const isValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  console.log(isValid ? 'Gültige E-Mail-Adresse' : 'Ungültige E-Mail-Adresse');
}
```

### 3. die Drag-and-Drop-Implementierung

Kombinieren Sie Mausereignisse, um Drag & Drop zu implementieren.

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, takeUntil, map } from 'rxjs';

// Ziehbare Elemente erstellen
const element = document.createElement('div');
element.style.width = '100px';
element.style.height = '100px';
element.style.backgroundColor = '#333';
element.style.position = 'absolute'; // Auf absolute Positionierung setzen
element.style.left = '50px'; // Ausgangsposition
element.style.top = '50px';
element.style.cursor = 'move'; // Ziehbarer Cursor
document.body.appendChild(element);

const mousedown$ = fromEvent<MouseEvent>(element, 'mousedown');
const mousemove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseup$ = fromEvent<MouseEvent>(document, 'mouseup');

mousedown$.pipe(
  switchMap(startEvent => {
    // Klickposition im Element erfassen
    const startX = startEvent.clientX - element.offsetLeft;
    const startY = startEvent.clientY - element.offsetTop;

    return mousemove$.pipe(
      map(moveEvent => ({
        left: moveEvent.clientX - startX,
        top: moveEvent.clientY - startY
      })),
      takeUntil(mouseup$) // Endet mit Maus hoch
    );
  })
).subscribe(({ left, top }) => {
  // Elementposition aktualisieren
  element.style.left = `${left}px`;
  element.style.top = `${top}px`;
});
```

### 4. Überwachung von Bildlaufereignissen

Dient zur Überwachung des unendlichen Scrollens und der Scrollposition.

```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Verwendung in der Pipeline.

fromEvent()` ist ideal für die Pipeline-Verarbeitung ausgehend von einem Ereignisstrom.


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Häufige Fehler.

### 1. Vergessen, sich abzumelden

#### ❌ Fehler - Vergessen der Abmeldung verursacht Speicherlecks

{\CODE_12___

#### ✅ Richtig - immer abmelden


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

> [!WARNING]

> **Achtung Speicherlecks**
>
> In SPA und komponentenbasierten Frameworks sollten Sie sich immer abmelden, wenn Sie eine Komponente zerstören. Wenn Sie die Abmeldung vergessen, bleiben die Ereignis-Listener bestehen und verursachen Speicherlecks.

### 2. doppelte Registrierung von mehreren Ereignis-Listenern

#### ❌ Fehler - das mehrfache Abonnieren desselben Ereignisses führt zur Registrierung mehrerer Listener.

{\CODE_14___

#### ✅ Richtig - Multicast mit share() falls erforderlich


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Leistungsüberlegungen.

Leistungserwägungen sollten berücksichtigt werden, wenn es um hochfrequente Auslöseereignisse geht (Scrollen, Mausbewegung, Größenänderung usw.).

> [!TIP]

> **Optimierung von hochfrequenten Ereignissen**:.
> - `throttleTime()` - nur einmal in einer bestimmten Zeitspanne behandeln.
> - `debounceTime()` - Verarbeitung nachdem die Eingabe gestoppt wurde.
> - `distinctUntilChanged()` - Nur verarbeiten, wenn sich der Wert ändert.

#### ❌ Leistungsprobleme - bei jeder Größenänderung verarbeiten


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

#### ✅ Optimierung - nur einmal alle 200ms verarbeitet


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Relevante Creation Function.


```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

## Zusammenfassung

- fromEvent()` konvertiert DOM Events und EventEmitter in Observable
- Listener wird registriert, wenn er abonniert wird, und automatisch gelöscht, wenn er abbestellt wird (verhindert Speicherlecks)
- Funktioniert wie ein Hot Observable
- Abmeldung immer durchführen, um Speicherlecks zu verhindern
- Optimieren Sie hochfrequente Ereignisse mit `throttleTime()` und `debounceTime()`.

## Nächste Schritte.

- [interval() - Werte in regelmäßigen Abständen ausgeben](/de/guide/creation-functions/basic/interval)
- timer() - Start der Veröffentlichung nach einer Verzögerung](/de/guide/creation-functions/basic/timer)
- [zurück zur Übersicht der grundlegenden Erstellungsfunktionen](/de/guide/creation-functions/basic/)
