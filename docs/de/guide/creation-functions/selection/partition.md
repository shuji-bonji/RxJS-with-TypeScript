---
description: "partition teilt ein Observable in zwei basierend auf Bedingungen. Nützlich für Erfolg/Fehler-Aufteilung mit typsicherem TypeScript."
---

# partition - Teilt in zwei Streams basierend auf einer Bedingung

`partition` ist eine Creation Function, die **ein Observable basierend auf einer Bedingung in zwei Observables aufteilt**.
Sie können Werte, die eine Prädikatfunktion erfüllen, und solche, die sie nicht erfüllen, als separate Streams erhalten.

## Grundlegende Syntax und Verwendung

```ts
import { partition, of } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// In gerade und ungerade aufteilen
const [evens$, odds$] = partition(source$, (value) => value % 2 === 0);

evens$.subscribe((value) => console.log('Gerade:', value));
// Ausgabe: Gerade: 2, Gerade: 4, Gerade: 6

odds$.subscribe((value) => console.log('Ungerade:', value));
// Ausgabe: Ungerade: 1, Ungerade: 3, Ungerade: 5
```

- `partition` gibt ein **Array mit zwei Observables** zurück
- `[0]`: Stream der Werte, die die Bedingung erfüllen
- `[1]`: Stream der Werte, die die Bedingung nicht erfüllen

[🌐 RxJS Offizielle Dokumentation - `partition`](https://rxjs.dev/api/index/function/partition)

## Typische Anwendungsmuster

- **Verzweigungsverarbeitung für Erfolg/Fehler** (Sortierung nach HTTP-Statuscodes)
- **Ereignissortierung** (Linksklick/Rechtsklick)
- **Datenklassifizierung** (gültig/ungültig, Erwachsene/Kinder usw.)
- **Bedingungsbasierte Stream-Aufteilung**

## Praktisches Codebeispiel (mit UI)

Bei einem Klick auf einen Button wird die Verarbeitung basierend darauf verzweigt, ob die Klickkoordinaten in der linken oder rechten Hälfte des Bildschirms liegen.

```ts
import { partition, fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Ausgabebereiche erstellen
const leftArea = document.createElement('div');
leftArea.innerHTML = '<h3>Linke Klicks</h3><ul id="left-list"></ul>';
leftArea.style.float = 'left';
leftArea.style.width = '45%';
leftArea.style.padding = '10px';
leftArea.style.background = '#e3f2fd';
document.body.appendChild(leftArea);

const rightArea = document.createElement('div');
rightArea.innerHTML = '<h3>Rechte Klicks</h3><ul id="right-list"></ul>';
rightArea.style.float = 'right';
rightArea.style.width = '45%';
rightArea.style.padding = '10px';
rightArea.style.background = '#fce4ec';
document.body.appendChild(rightArea);

// Klick-Ereignis
const clicks$ = fromEvent<MouseEvent>(document, 'click');

// Zentrale X-Koordinate des Bildschirms
const centerX = window.innerWidth / 2;

// In linke und rechte Hälfte aufteilen
const [leftClicks$, rightClicks$] = partition(
  clicks$,
  (event) => event.clientX < centerX
);

// Linke Klicks verarbeiten
leftClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const leftList = document.getElementById('left-list')!;
  const li = document.createElement('li');
  li.textContent = `Koordinaten: (${pos.x}, ${pos.y})`;
  leftList.appendChild(li);
});

// Rechte Klicks verarbeiten
rightClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const rightList = document.getElementById('right-list')!;
  const li = document.createElement('li');
  li.textContent = `Koordinaten: (${pos.x}, ${pos.y})`;
  rightList.appendChild(li);
});
```

- Bei einem Klick auf den Bildschirm wird die Klickposition je nach Position in der linken oder rechten Liste aufgezeichnet.
- Sie können zwei unabhängige Streams aus einer Quelle erstellen.

## Praktisches Beispiel: Verzweigungsverarbeitung bei API-Antworten

Beispiel für die Aufteilung nach HTTP-Statuscode in Erfolg und Fehler

```ts
import { partition, from, of } from 'rxjs';
import { mergeMap, map, catchError, share } from 'rxjs';

interface ApiResponse {
  status: number;
  data?: any;
  error?: string;
}

// Dummy-API-Aufrufe
const apiCalls$ = from([
  fetch('/api/users/1'),
  fetch('/api/users/999'), // Nicht existierender Benutzer
  fetch('/api/users/2'),
]);

// Response verarbeiten und in ApiResponse umwandeln
const responses$ = apiCalls$.pipe(
  mergeMap(fetchPromise => from(fetchPromise)),
  mergeMap(response =>
    from(response.json()).pipe(
      map(data => ({
        status: response.status,
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : (data.message || 'Error')
      } as ApiResponse)),
      catchError(err => of({
        status: response.status,
        data: undefined,
        error: err.message || 'Failed to parse response'
      } as ApiResponse))
    )
  ),
  share() // Für zweifaches Abonnieren bei partition
);

// In Erfolg (200er) und Fehler (andere) aufteilen
const [success$, failure$] = partition(
  responses$,
  (response: ApiResponse) => response.status >= 200 && response.status < 300
);

// Erfolgreiche Antworten verarbeiten
success$.subscribe((response) => {
  console.log('✅ Erfolg:', response.data);
  // Erfolgsdaten in UI anzeigen
});

// Fehlerhafte Antworten verarbeiten
failure$.subscribe((response) => {
  console.error('❌ Fehler:', response.error);
  // Fehlermeldung anzeigen
});
```

## Vergleich mit filter

### Grundlegender Unterschied

| Methode | Beschreibung | Ausgabe | Anwendungsfall |
|---------|--------------|---------|----------------|
| `partition` | Teilt eine Quelle in zwei Streams auf | Zwei Observables | Wenn Sie beide Streams **gleichzeitig** verwenden möchten |
| `filter` | Lässt nur Werte durch, die die Bedingung erfüllen | Ein Observable | Wenn nur ein Stream benötigt wird |

### Konkrete Beispiele für die Verwendung

**Verwenden Sie partition, wenn Sie beide Streams gleichzeitig verarbeiten**

```ts
import { partition, interval } from 'rxjs';
import { map, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Erfolg</h4><ul id="success-list"></ul>';
successArea.style.float = 'left';
successArea.style.width = '45%';
output.appendChild(successArea);

const failureArea = document.createElement('div');
failureArea.innerHTML = '<h4 style="color: red;">❌ Fehler</h4><ul id="failure-list"></ul>';
failureArea.style.float = 'right';
failureArea.style.width = '45%';
output.appendChild(failureArea);

// Zufälliger Erfolg/Fehler-Stream
const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Aufgabe ${i + 1}`
  }))
);

// ✅ partition - Erfolg und Fehler gleichzeitig verarbeiten
const [success$, failure$] = partition(tasks$, task => task.success);

success$.subscribe(task => {
  const successList = document.getElementById('success-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  successList.appendChild(li);
});

failure$.subscribe(task => {
  const failureList = document.getElementById('failure-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  failureList.appendChild(li);
});
```

**Verwenden Sie filter, wenn nur ein Stream benötigt wird**

```ts
import { interval } from 'rxjs';
import { map, take, filter } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Nur Erfolg anzeigen</h4><ul id="success-only"></ul>';
output.appendChild(successArea);

const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Aufgabe ${i + 1}`
  }))
);

// ✅ filter - Nur Erfolg verarbeiten (Fehler werden ignoriert)
tasks$
  .pipe(filter(task => task.success))
  .subscribe(task => {
    const successList = document.getElementById('success-only')!;
    const li = document.createElement('li');
    li.textContent = task.message;
    successList.appendChild(li);
  });
```

**Vergleich von filter zweimal verwenden vs. partition**

```ts
import { of } from 'rxjs';
import { filter } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// ❌ filter zweimal verwenden - Quelle könnte zweimal ausgeführt werden
const evens1$ = numbers$.pipe(filter(n => n % 2 === 0));
const odds1$ = numbers$.pipe(filter(n => n % 2 !== 0));

evens1$.subscribe(n => console.log('Gerade:', n));
odds1$.subscribe(n => console.log('Ungerade:', n));
// Problem: Wenn numbers$ ein Cold Observable ist, wird es zweimal ausgeführt

// ✅ partition verwenden - Erstellt beide Streams aus einer Ausführung
const [evens2$, odds2$] = partition(numbers$, n => n % 2 === 0);

evens2$.subscribe(n => console.log('Gerade:', n));
odds2$.subscribe(n => console.log('Ungerade:', n));
// Vorteil: Erstellt effizient zwei Streams aus einer Quelle
```

**Verwenden Sie filter, wenn Sie in einer Pipeline verzweigen möchten**

```ts
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

interface User {
  id: number;
  name: string;
  age: number;
  isActive: boolean;
}

const users$ = from([
  { id: 1, name: 'Alice', age: 25, isActive: true },
  { id: 2, name: 'Bob', age: 30, isActive: false },
  { id: 3, name: 'Carol', age: 35, isActive: true }
]);

// ❌ partition ist eine Creation Function und kann nicht in einer Pipeline verwendet werden
// users$.pipe(
//   map(user => user.name),
//   partition(name => name.startsWith('A')) // Fehler
// );

// ✅ filter verwenden - Kann in Pipeline verwendet werden
users$
  .pipe(
    filter(user => user.isActive),  // Nur aktive Benutzer
    map(user => user.name)           // Namen extrahieren
  )
  .subscribe(console.log);
// Ausgabe: Alice, Carol
```

### Zusammenfassung

| Situation | Empfohlene Methode | Grund |
|-----------|-------------------|-------|
| Sowohl Erfolg **als auch** Fehler verarbeiten | `partition` | Kann zwei Streams aus einer Quellausführung erstellen |
| Nur Erfolg **verarbeiten** | `filter` | Einfach und verständlich |
| In Pipeline verzweigen | `filter` | `partition` ist eine Creation Function und kann nicht verwendet werden |
| Verzweigung in 3 oder mehr mit komplexen Bedingungen | `groupBy` | Kann in mehrere Gruppen aufteilen |

## Wichtige Hinweise

### 1. Beide Streams abonnieren

Die beiden von `partition` erstellten Observables **teilen die ursprüngliche Quelle**.
Wenn Sie nicht beide abonnieren, wird der ursprüngliche Stream möglicherweise nicht vollständig verarbeitet.

```ts
const [success$, failure$] = partition(source$, predicate);

// Beide abonnieren
success$.subscribe(handleSuccess);
failure$.subscribe(handleFailure);
```

### 2. Quelle wird zweimal ausgeführt

`partition` abonniert die ursprüngliche Quelle intern zweimal.
Seien Sie vorsichtig bei Nebeneffekten.

```ts
let callCount = 0;
const source$ = new Observable(observer => {
  callCount++;
  console.log(`Abonnement-Anzahl: ${callCount}`);
  observer.next(1);
  observer.complete();
});

const [a$, b$] = partition(source$, n => n > 0);
a$.subscribe(); // Abonnement-Anzahl: 1
b$.subscribe(); // Abonnement-Anzahl: 2
```

Um Nebeneffekte zu vermeiden, verwenden Sie `share()`.

```ts
import { share } from 'rxjs';

const shared$ = source$.pipe(share());
const [a$, b$] = partition(shared$, n => n > 0);
```

### 3. Nicht als Pipeable Operator verfügbar

Ab RxJS 7 ist `partition` **nur als Creation Function** verfügbar.
Es kann nicht in einer Pipeline verwendet werden.

```ts
// ❌ Nicht möglich
source$.pipe(
  partition(n => n % 2 === 0) // Fehler
);

// ✅ Korrekte Verwendung
const [evens$, odds$] = partition(source$, n => n % 2 === 0);
```

## Alternative Muster

Wenn Sie in einer Pipeline verzweigen möchten, verwenden Sie `filter`.

```ts
const source$ = of(1, 2, 3, 4, 5, 6);

const evens$ = source$.pipe(filter(n => n % 2 === 0));
const odds$ = source$.pipe(filter(n => n % 2 !== 0));

// Oder teilen Sie die Quelle mit share
const shared$ = source$.pipe(share());
const evens$ = shared$.pipe(filter(n => n % 2 === 0));
const odds$ = shared$.pipe(filter(n => n % 2 !== 0));
```

## Verwandte Operatoren

- [`filter`](../../operators/filtering/filter.md) - Lässt nur Werte durch, die die Bedingung erfüllen
- [`groupBy`](../../operators/transformation/groupBy.md) - Teilt in mehrere Gruppen auf
- [`share`](../../operators/multicasting/share.md) - Teilt die Quelle

## Zusammenfassung

`partition` ist ein leistungsstarkes Werkzeug zum Aufteilen eines Observable in zwei basierend auf einer Bedingung.

- ✅ Optimal für Verzweigungsverarbeitung bei Erfolg/Fehler
- ✅ Erstellt zwei unabhängige Streams
- ⚠️ Quelle wird zweimal abonniert (Vorsicht bei Nebeneffekten)
- ⚠️ Nicht als Pipeable Operator verfügbar
