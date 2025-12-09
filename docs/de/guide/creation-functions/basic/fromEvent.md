---
description: "fromEvent() - Erstellungsfunktion zur Konvertierung von DOM-Ereignissen und EventEmittern in Observables, die Grundlage der ereignisgesteuerten Programmierung, für verschiedene Ereignisverarbeitungen wie Klick, Tastendruck, Mausbewegung, Scrollen usw."
---

# fromEvent() - Konvertiert ein Ereignis in Observable

`fromEvent()` ist eine Erstellungsfunktion, die Ereignisquellen wie DOM-Ereignisse und Node.js EventEmitter in Observable-Streams umwandelt.

## Überblick

`fromEvent()` ermöglicht eine ereignisbasierte asynchrone Verarbeitung in der RxJS-Pipeline. Es registriert automatisch Event-Listener, wenn sie abonniert werden, und entfernt automatisch Listener, wenn sie abbestellt werden, wodurch das Risiko von Speicherlecks erheblich reduziert wird.

**Signatur**:
```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

**Offizielle Dokumentation**: [📘 RxJS Official: fromEvent()](https://rxjs.dev/api/index/function/fromEvent)

## Grundlegende Verwendung

Dies ist das einfachste Beispiel für die Behandlung von DOM-Ereignissen als Observable.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Button wurde geklickt:', event);
});

// Jedes Mal, wenn geklickt wird, wird ein Ereignis ausgegeben
```

## Wichtige Merkmale

### 1. Automatische Registrierung und Deregistrierung von Listenern

Die Funktion `fromEvent()` registriert einen Ereignis-Listener beim Abonnieren und entfernt ihn automatisch beim Abbestellen.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent<MouseEvent>(document, 'click');

const subscription = clicks$.subscribe(event => {
  console.log('Klickposition:', event.clientX, event.clientY);
});

// Nach 5 Sekunden abbestellen (Event-Listener wird automatisch entfernt)
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Abbestellt');
}, 5000);
```

> [!IMPORTANT]
> **Vermeidung von Speicherlecks**
>
> Beim Aufruf von `unsubscribe()` wird intern automatisch `removeEventListener()` ausgeführt. Dadurch entfällt die Notwendigkeit, Listener manuell zu entfernen, und das Risiko von Speicherlecks wird erheblich reduziert.

### 2. Cold Observable (jedes Abonnement registriert einen unabhängigen Listener)

Die durch `fromEvent()` erstellte Observable ist eine **Cold Observable**. Jedes Abonnement registriert einen unabhängigen Ereignis-Listener.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Abonnement 1 - Listener A wird registriert
clicks$.subscribe(() => console.log('Observer 1: Klick'));

// 1 Sekunde später Abonnement 2 hinzufügen - Listener B wird unabhängig registriert
setTimeout(() => {
  clicks$.subscribe(() => console.log('Observer 2: Klick'));
}, 1000);

// Bei einem Klick werden beide Listener ausgelöst
```

> [!NOTE]
> **Cold Observable Merkmale**
> - Jedes Abonnement startet eine unabhängige Ausführung
> - Jeder Abonnent erhält seinen eigenen Datenstrom
> - Bei jedem Abonnement wird ein unabhängiger Event-Listener registriert. Bei unsubscribe wird der Listener automatisch entfernt.
>
> Weitere Informationen finden Sie unter [Cold Observable und Hot Observable](/de/guide/observables/cold-and-hot-observables).

### 3. TypeScript Typ-Unterstützung

Ereignistypen können explizit angegeben werden.

```typescript
import { fromEvent } from 'rxjs';

const input = document.createElement('input');
input.type = 'text';
document.body.appendChild(input);
const input$ = fromEvent<InputEvent>(input, 'input');

input$.subscribe(event => {
  // Der Typ von event ist InputEvent
  const target = event.target as HTMLInputElement;
  console.log('Eingabewert:', target.value);
});
```

## Praktische Anwendungsfälle

### 1. Klickereignis-Verarbeitung

Steuert Button-Klicks und verhindert aufeinanderfolgende Klicks.

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const button = document.createElement('button');
button.innerText = "submit";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  debounceTime(300), // Aufeinanderfolgende Klicks innerhalb von 300ms ignorieren
  map(() => 'Wird gesendet...')
).subscribe(message => {
  console.log(message);
  // API-Aufruf usw.
});
```

### 2. Echtzeit-Formularvalidierung

Streamt Eingabeereignisse und führt Echtzeit-Validierung durch.

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
  debounceTime(500), // 500ms nach dem Ende der Eingabe verarbeiten
  distinctUntilChanged() // Nur wenn sich der Wert ändert
).subscribe(email => {
  console.log('Zu validieren:', email);
  validateEmail(email);
});

function validateEmail(email: string): void {
  const isValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  console.log(isValid ? 'Gültige E-Mail-Adresse' : 'Ungültige E-Mail-Adresse');
}
```

### 3. Drag & Drop Implementierung

Kombiniert Mausereignisse zur Implementierung von Drag & Drop.

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, takeUntil, map } from 'rxjs';

// Ziehbares Element erstellen
const element = document.createElement('div');
element.style.width = '100px';
element.style.height = '100px';
element.style.backgroundColor = '#333';
element.style.position = 'absolute';
element.style.left = '50px';
element.style.top = '50px';
element.style.cursor = 'move';
document.body.appendChild(element);

const mousedown$ = fromEvent<MouseEvent>(element, 'mousedown');
const mousemove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseup$ = fromEvent<MouseEvent>(document, 'mouseup');

mousedown$.pipe(
  switchMap(startEvent => {
    const startX = startEvent.clientX - element.offsetLeft;
    const startY = startEvent.clientY - element.offsetTop;

    return mousemove$.pipe(
      map(moveEvent => ({
        left: moveEvent.clientX - startX,
        top: moveEvent.clientY - startY
      })),
      takeUntil(mouseup$)
    );
  })
).subscribe(({ left, top }) => {
  element.style.left = `${left}px`;
  element.style.top = `${top}px`;
});
```

### 4. Scroll-Ereignis-Überwachung

Wird für unendliches Scrollen oder Scroll-Position-Tracking verwendet.

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

const scroll$ = fromEvent(window, 'scroll');

scroll$.pipe(
  throttleTime(200), // Nur einmal alle 200ms verarbeiten
  map(() => window.scrollY)
).subscribe(scrollPosition => {
  console.log('Scroll-Position:', scrollPosition);

  // Zusätzlichen Inhalt laden, wenn das Ende der Seite erreicht ist
  if (scrollPosition + window.innerHeight >= document.body.scrollHeight - 100) {
    console.log('Zusätzlichen Inhalt laden');
  }
});
```

## Häufige Fehler

### 1. Vergessen abzubestellen

```typescript
// ❌ Falsch - Vergessen abzubestellen führt zu Speicherleck
import { fromEvent } from 'rxjs';

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  clicks$.subscribe(console.log); // Wird nie abbestellt!
}

setupEventListener();

// ✅ Richtig - Immer abbestellen
import { fromEvent, Subscription } from 'rxjs';

let subscription: Subscription;

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  subscription = clicks$.subscribe(console.log);
}

function cleanup() {
  if (subscription) {
    subscription.unsubscribe();
  }
}

setupEventListener();
// cleanup() bei Komponenten-Zerstörung aufrufen
```

> [!WARNING]
> **Vorsicht vor Speicherlecks**
>
> Bei SPAs und komponentenbasierten Frameworks immer abbestellen, wenn die Komponente zerstört wird. Das Vergessen des Abbestellens führt zu verbleibenden Event-Listenern und Speicherlecks.

### 2. Mehrfache Event-Listener registrieren

```typescript
// ❌ Falsch - Mehrfaches Abonnieren desselben Events erstellt mehrere Listener
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Observer 1'));
clicks$.subscribe(() => console.log('Observer 2'));
// Beim Klicken werden beide Logs angezeigt (2 Listener sind registriert)

// ✅ Richtig - Bei Bedarf mit share() multicasten
import { fromEvent } from 'rxjs';
import { share } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(share());

clicks$.subscribe(() => console.log('Observer 1'));
clicks$.subscribe(() => console.log('Observer 2'));
// Ein Listener wird geteilt
```

## Leistungsüberlegungen

Bei hochfrequenten Ereignissen (scroll, mousemove, resize usw.) ist Vorsicht bei der Leistung geboten.

> [!TIP]
> **Optimierung hochfrequenter Ereignisse**:
> - `throttleTime()` - Nur einmal pro Zeitintervall verarbeiten
> - `debounceTime()` - Nach Ende der Eingabe verarbeiten
> - `distinctUntilChanged()` - Nur verarbeiten wenn sich der Wert ändert

```typescript
// ❌ Leistungsproblem - Bei jeder Größenänderung verarbeiten
import { fromEvent } from 'rxjs';

const resize$ = fromEvent(window, 'resize');
resize$.subscribe(() => {
  console.log('Größenänderungsverarbeitung'); // Hohe Last
});

// ✅ Optimierung - Nur einmal alle 200ms verarbeiten
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

const resize$ = fromEvent(window, 'resize');
resize$.pipe(
  throttleTime(200)
).subscribe(() => {
  console.log('Größenänderungsverarbeitung'); // Reduzierte Last
});
```

## Verwandte Erstellungsfunktionen

| Funktion | Unterschiede | Verwendung |
|----------|------|----------|
| **[from()](/de/guide/creation-functions/basic/from)** | Konvertiert von Array/Promise | Streamt nicht-Event-Daten |
| **[interval()](/de/guide/creation-functions/basic/interval)** | Ausgabe in regelmäßigen Abständen | Wenn regelmäßige Verarbeitung benötigt wird |
| **fromEventPattern()** | Benutzerdefinierte Event-Registrierung | Eigene Event-Systeme außer EventEmitter |

## Zusammenfassung

- `fromEvent()` konvertiert DOM-Ereignisse und EventEmitter in Observable
- Listener werden beim Abonnieren registriert und beim Abbestellen automatisch entfernt (verhindert Speicherlecks)
- Funktioniert als Hot Observable
- Immer abbestellen um Speicherlecks zu verhindern
- Hochfrequente Ereignisse mit `throttleTime()` oder `debounceTime()` optimieren

## Nächste Schritte

- [interval() - Werte in regelmäßigen Abständen ausgeben](/de/guide/creation-functions/basic/interval)
- [timer() - Start nach Verzögerung](/de/guide/creation-functions/basic/timer)
- [Zurück zu Grundlegende Erstellungsfunktionen](/de/guide/creation-functions/basic/)
