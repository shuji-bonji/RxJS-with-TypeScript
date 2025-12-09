---
description: "Verstehen Sie den Unterschied zwischen Promise und RxJS und lernen Sie die richtige Verwendung. Promise ist auf einzelne asynchrone Verarbeitungen spezialisiert und wird sofort ausgeführt, während RxJS Lazy-Evaluations-Stream-Verarbeitung bietet, die mehrere Werte handhaben kann. Vergleich von Abbruch, Wiederholung und Transformation mit TypeScript-Erläuterungen."
---

# Unterschied zwischen Promise und RxJS

## Übersicht

Als Hauptwerkzeuge zur Behandlung asynchroner Verarbeitungen in JavaScript/TypeScript gibt es **Promise** und **RxJS (Observable)**. Obwohl beide für ähnliche Zwecke verwendet werden, unterscheiden sich ihre Designphilosophie und Anwendungsfälle erheblich.

Diese Seite bietet Informationen zum Verständnis der Unterschiede zwischen Promise und RxJS und zur Entscheidung, welche verwendet werden sollte.

## Grundlegende Unterschiede

| Element | Promise | RxJS (Observable) |
|------|---------|-------------------|
| **Standardisierung** | JavaScript-Standard (ES6/ES2015) | Drittanbieter-Bibliothek |
| **Emittierte Werte** | Einzelner Wert | 0 oder mehr mehrere Werte |
| **Auswertung** | Eager (sofortige Ausführung bei Erstellung) | Lazy (Ausführung bei Abonnement) |
| **Abbruch** | Nicht möglich[^1] | Möglich (`unsubscribe()`) |
| **Wiederverwendung** | Nicht möglich (Ergebnis nur einmal) | Möglich (beliebig oft abonnierbar) |
| **Lernkosten** | Niedrig | Hoch (Verständnis der Operatoren erforderlich) |
| **Anwendungsfall** | Einzelne asynchrone Verarbeitung | Komplexe Stream-Verarbeitung |

[^1]: Mit AbortController kann Promise-basierte Verarbeitung (wie fetch) abgebrochen werden, aber die Promise-Spezifikation selbst hat keine Abbruchfunktion.

## Code-Vergleich: Einzelne asynchrone Verarbeitung

### Promise

```ts
// Promise wird bei Erstellung sofort ausgeführt (Eager)
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json())
  .then(data => console.log(data))
  .catch(error => console.error(error));
```

Promise **beginnt mit der Ausführung in dem Moment, in dem es definiert wird** (Eager-Auswertung).

### RxJS

```ts
import { from } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

// Observable wird nicht ausgeführt, bis es abonniert wird (Lazy)
const observable$ = from(fetch('https://jsonplaceholder.typicode.com/posts/1')).pipe(
  switchMap(response => response.json()), // response.json() gibt Promise zurück, daher switchMap verwenden
  catchError(error => {
    console.error(error);
    return of(null);
  })
);

// Wird erst bei Abonnement ausgeführt
observable$.subscribe(data => console.log(data));
```

RxJS **wird nicht ausgeführt, bis `subscribe()` aufgerufen wird** (Lazy-Auswertung). Wenn dasselbe Observable mehrmals abonniert wird, werden unabhängige Ausführungen durchgeführt, und die Verarbeitung kann mit `unsubscribe()` unterbrochen werden.

> [!TIP]
> **Verwendung in der Praxis**
> - Sofort auszuführende Einzelverarbeitung → Promise
> - Zum gewünschten Zeitpunkt oder mehrmals auszuführende Verarbeitung → RxJS

## Code-Vergleich: Umgang mit mehreren Werten

Einer der größten Unterschiede zwischen Promise und RxJS ist die Anzahl der Werte, die emittiert werden können. Promise kann nur einen einzelnen Wert zurückgeben, während RxJS mehrere Werte zeitlich emittieren kann.

### Mit Promise nicht möglich

Promise **kann nur einmal aufgelöst werden**.

```ts
// Promise kann nur einen einzelnen Wert zurückgeben
const promise = new Promise(resolve => {
  resolve(1);
  resolve(2); // Dieser Wert wird ignoriert
  resolve(3); // Dieser Wert wird ebenfalls ignoriert
});

promise.then(value => console.log(value));
// Ausgabe: 1 (nur der erste Wert)
```

Sobald der Wert mit dem ersten `resolve()` festgelegt ist, werden alle nachfolgenden `resolve()` ignoriert.

### Mit RxJS möglich

Observable **kann beliebig oft Werte emittieren**.
```ts
import { Observable } from 'rxjs';

// Observable kann mehrere Werte emittieren
const observable$ = new Observable(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  subscriber.complete();
});

observable$.subscribe(value => console.log(value));
// Ausgabe: 1, 2, 3
```

Jedes Mal, wenn `next()` aufgerufen wird, erreicht ein Wert den Abonnenten. Nach dem Emittieren aller Werte wird die Vollständigkeit mit `complete()` signalisiert. Diese Eigenschaft ermöglicht die natürliche Handhabung von zeitlich veränderlichen Daten wie Echtzeit-Kommunikation, Streaming-Daten und kontinuierlicher Ereignisverarbeitung.

> [!NOTE]
> **Anwendungsbeispiele in der Praxis**
> - WebSocket-Nachrichtenempfang
> - Sequenzielle Verarbeitung von Tastatureingaben
> - Event-Streams vom Server (SSE)
> - Kontinuierliche Überwachung von Sensordaten

## Abbruch-Vergleich

Ob lang laufende Verarbeitungen oder unnötig gewordene asynchrone Verarbeitungen abgebrochen werden können, ist aus Sicht des Ressourcenmanagements und der Benutzererfahrung wichtig. Zwischen Promise und RxJS gibt es große Unterschiede in der Abbruchfunktion.

### Promise (nicht abbrechbar)
Promise hat **keine standardmäßige Abbruchfunktion**.

```ts
const promise = new Promise(resolve => {
  setTimeout(() => resolve('Abgeschlossen'), 3000);
});

promise.then(result => console.log(result));
// Es gibt keine standardmäßige Methode, diese Verarbeitung abzubrechen
```

Einmal gestartet, kann die Verarbeitung nicht gestoppt werden, bis sie abgeschlossen ist, was zu Speicherlecks und Leistungsproblemen führen kann.

> [!WARNING]
> **Über AbortController**
> Web-APIs wie `fetch()` können mit `AbortController` abgebrochen werden, aber dies ist keine Funktion von Promise selbst, sondern ein Mechanismus, der von einzelnen APIs bereitgestellt wird. Er ist nicht für alle asynchronen Verarbeitungen verfügbar.

### RxJS (abbrechbar)

RxJS **kann jederzeit mit `unsubscribe()` abgebrochen werden**.
```ts
import { timer } from 'rxjs';

const subscription = timer(3000).subscribe(
  () => console.log('Abgeschlossen')
);

// Nach 1 Sekunde abbrechen
setTimeout(() => {
  subscription.unsubscribe(); // Abbruch
  console.log('Abgebrochen');
}, 1000);
// Ausgabe: Abgebrochen ("Abgeschlossen" wird nicht ausgegeben)
```

Wenn das Abonnement aufgehoben wird, stoppt die laufende Verarbeitung sofort und Speicherlecks werden verhindert.

> [!TIP]
> **Abbruch-Anwendungsbeispiele in der Praxis**
> - HTTP-Anfragen abbrechen, wenn der Benutzer die Seite verlässt
> - Alte Suchanfrageergebnisse verwerfen und nur die neueste Anfrage verarbeiten (`switchMap`)
> - Alle Observables automatisch abbrechen, wenn Komponenten zerstört werden (`takeUntil`-Muster)

## Welche sollte gewählt werden?

Welche von Promise und RxJS verwendet werden sollte, hängt von der Art der Verarbeitung und den Projektanforderungen ab. Wählen Sie das geeignete Werkzeug anhand der folgenden Kriterien.

### Wann Promise gewählt werden sollte

Wenn die folgenden Bedingungen zutreffen, ist Promise geeignet.

| Bedingung | Grund |
|------|------|
| Einzelne asynchrone Verarbeitung | 1 API-Anfrage, 1 Dateileseoperation etc. |
| Einfacher Workflow | `Promise.all`, `Promise.race` sind ausreichend |
| Kleines Projekt | Abhängigkeiten minimieren |
| Nur Standard-API verwenden | Externe Bibliotheken vermeiden |
| Code für Anfänger | Lernkosten niedrig halten |

#### Einzelne API-Anfrage:


```ts
interface User {
  id: number;
  name: string;
  email: string;
  username: string;
}

async function getUserData(userId: string): Promise<User> {
  const response = await fetch(`https://jsonplaceholder.typicode.com/users/${userId}`);
  if (!response.ok) {
    throw new Error('Benutzerdaten konnten nicht abgerufen werden');
  }
  return response.json();
}

// Verwendungsbeispiel
getUserData('1').then(user => {
  console.log('Benutzername:', user.name);
  console.log('E-Mail:', user.email);
});
```

Dieser Code ist ein typisches Muster zum Abrufen einzelner Benutzerinformationen. Mit `async/await` kann lesbar wie synchroner Code geschrieben werden. Auch die Fehlerbehandlung ist mit `try/catch` einheitlich und einfach und intuitiv.

#### Parallele Ausführung mehrerer asynchroner Verarbeitungen:

```ts
interface Post {
  id: number;
  userId: number;
  title: string;
  body: string;
}

async function loadAllData(): Promise<[User[], Post[]]> {
  const [users, posts] = await Promise.all([
    fetch('https://jsonplaceholder.typicode.com/users').then(r => r.json()),
    fetch('https://jsonplaceholder.typicode.com/posts').then(r => r.json())
  ]);
  return [users, posts];
}

// Verwendungsbeispiel
loadAllData().then(([users, posts]) => {
  console.log('Benutzeranzahl:', users.length);
  console.log('Beitragsanzahl:', posts.length);
});
```

Mit `Promise.all()` können mehrere API-Anfragen parallel ausgeführt und auf die Fertigstellung aller gewartet werden. Dies ist sehr praktisch für das Laden von Initialdaten. Zu beachten ist, dass bei einem Fehler die gesamte Verarbeitung fehlschlägt, aber gerade diese Einfachheit macht es leicht verständlich und wartbar.

### Wann RxJS gewählt werden sollte

Wenn die folgenden Bedingungen zutreffen, ist RxJS geeignet.

| Bedingung | Grund |
|------|------|
| Kontinuierliche Ereignisverarbeitung | Mausbewegung, Tastatureingabe, WebSocket etc. |
| Komplexe Stream-Verarbeitung | Kombination und Transformation mehrerer Ereignisquellen |
| Abbruch erforderlich | Feinere Ressourcenverwaltung gewünscht |
| Wiederholung/Timeout | Flexible Fehlerbehandlung gewünscht |
| Angular-Projekt | RxJS ist im Framework integriert |
| Echtzeitdaten | Daten werden kontinuierlich aktualisiert |

#### Konkretes Beispiel
```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'Suche: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);

// Echtzeit-Suche (Autovervollständigung)
if (!searchInput) throw new Error('Sucheingabefeld nicht gefunden');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),              // 300ms warten, dann verarbeiten
  distinctUntilChanged(),         // Nur verarbeiten, wenn sich der Wert geändert hat
  switchMap(query =>              // Nur die neueste Anfrage ausführen
    fetch(`https://api.github.com/search/users?q=${query}`).then(r => r.json())
  )
).subscribe(results => {
  console.log('Suchergebnisse:', results.items); // GitHub API speichert Ergebnisse im items-Property
});
```

Dieses Beispiel ist ein typischer Fall, in dem RxJS seinen wahren Wert zeigt. Die Benutzereingabe wird überwacht, eine Wartezeit von 300ms wird eingerichtet, um unnötige Anfragen zu reduzieren, die Verarbeitung erfolgt nur bei Wertänderungen, und durch Aktivierung nur der neuesten Anfrage (`switchMap`) werden alte Anfrageergebnisse automatisch verworfen.

> [!IMPORTANT]
> **Warum es mit Promise allein schwierig ist**
> - Debounce (Steuerung kontinuierlicher Eingaben) muss manuell implementiert werden
> - Abbruch alter Anfragen muss selbst verwaltet werden
> - Vergessenes Aufräumen von Event-Listenern führt zu Speicherlecks
> - Mehrere Zustände (Timer, Flags, Anfragenverwaltung) müssen gleichzeitig verfolgt werden
>
> Mit RxJS kann all dies deklarativ in wenigen Zeilen realisiert werden.

## Interoperabilität zwischen Promise und RxJS

Promise und RxJS schließen sich nicht gegenseitig aus und können ineinander umgewandelt und kombiniert werden. Dies ist praktisch, wenn vorhandener Promise-basierter Code in eine RxJS-Pipeline integriert werden soll oder umgekehrt Observables in vorhandenem Promise-basierten Code verwendet werden sollen.

## Umwandlung von Promise in Observable

RxJS bietet mehrere Methoden zur Umwandlung vorhandener Promises in Observables.

### Umwandlung mit `from`

Die gebräuchlichste Methode ist die Verwendung von `from`.

```ts
import { from } from 'rxjs';

// Promise erstellen
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json());

// Mit from() in Observable umwandeln
const observable$ = from(promise);

observable$.subscribe({
  next: data => console.log('Daten:', data),
  error: error => console.error('Fehler:', error),
  complete: () => console.log('Abgeschlossen')
});
```

`from()` emittiert einen Wert, wenn das Promise aufgelöst wird, und schließt sofort mit `complete` ab. Bei einem Fehler wird eine `error`-Benachrichtigung gesendet. Durch diese Umwandlung können RxJS-Operatoren (`map`, `filter`, `retry` etc.) frei auf Promise-Daten angewendet werden.

### Umwandlung mit `defer` (verzögerte Auswertung)

`defer` verzögert die Erstellung des Promise, bis es abonniert wird.

```ts
import { defer } from 'rxjs';

// Promise wird erst erstellt, wenn subscribe aufgerufen wird
const observable$ = defer(() =>
  fetch('https://jsonplaceholder.typicode.com/posts/1').then(r => r.json())
);

// Neues Promise wird bei jedem Abonnement erstellt
observable$.subscribe(data => console.log('1. Mal:', data));
observable$.subscribe(data => console.log('2. Mal:', data));
```

Diese Methode ist praktisch, wenn bei jedem Abonnement ein neues Promise erstellt werden soll.

## Umwandlung von Observable in Promise

Von einem Observable kann nur ein Wert extrahiert und in ein Promise umgewandelt werden.

### `firstValueFrom` und `lastValueFrom`

Ab RxJS 7 werden die folgenden zwei Funktionen empfohlen.

| Funktion | Verhalten |
|------|------|
| `firstValueFrom` | Gibt den ersten Wert als Promise zurück |
| `lastValueFrom` | Gibt den letzten Wert bei Abschluss als Promise zurück |

```ts
import { of, firstValueFrom, lastValueFrom } from 'rxjs';
import { delay } from 'rxjs';

const observable$ = of(1, 2, 3).pipe(delay(1000));

// Ersten Wert als Promise abrufen
const firstValue = await firstValueFrom(observable$);
console.log(firstValue); // 1

// Letzten Wert als Promise abrufen
const lastValue = await lastValueFrom(observable$);
console.log(lastValue); // 3
```

Wenn das Observable abschließt, bevor ein Wert emittiert wird, tritt standardmäßig ein Fehler auf. Dies kann durch Angabe eines Standardwerts vermieden werden.

> [!WARNING]
> `toPromise()` ist veraltet. Verwenden Sie stattdessen `firstValueFrom()` oder `lastValueFrom()`.

> [!TIP]
> **Auswahlrichtlinien**
> - **`firstValueFrom()`**: Wenn nur der erste Wert benötigt wird (z.B. Login-Authentifizierungsergebnis)
> - **`lastValueFrom()`**: Wenn das Endergebnis nach Verarbeitung aller Daten benötigt wird (z.B. Aggregationsergebnis)

## Praxisbeispiel: Kombination beider

In echten Anwendungen ist die kombinierte Verwendung von Promise und RxJS üblich.

> [!WARNING] Hinweis für die Praxis
> Die Vermischung von Promise und Observable kann **leicht zu Anti-Patterns führen, wenn die Designgrenzen nicht klar sind**.
>
> **Häufige Probleme:**
> - Nicht abbrechbar werden
> - Trennung der Fehlerbehandlung
> - `await` innerhalb von `subscribe` (besonders gefährlich)
> - Paralleles Abrufen derselben Daten mit Promise und Observable
>
> Weitere Details finden Sie unter **[Kapitel 10: Anti-Pattern der Promise- und Observable-Vermischung](/de/guide/anti-patterns/promise-observable-mixing)**.

### Formularübermittlung und API-Aufruf

Beispiel zum Erfassen von Formularübermittlungsereignissen mit RxJS und Senden an den Server mit Fetch API (Promise).

```ts
import { fromEvent, from } from 'rxjs';
import { exhaustMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface FormData {
  username: string;
  email: string;
}

// Promise-basierte Formularübermittlung
async function submitForm(data: FormData): Promise<{ success: boolean }> {
  const response = await fetch('https://api.example.com/submit', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  });
  if (!response.ok) {
    throw new Error('Übermittlung fehlgeschlagen');
  }
  return response.json();
}

// Event-Stream mit RxJS verwalten
const submitButton = document.createElement('button');
submitButton.id = 'submit-button';
submitButton.innerText = 'Senden';
submitButton.style.padding = '10px 20px';
submitButton.style.margin = '10px';
document.body.appendChild(submitButton);
if (!submitButton) throw new Error('Senden-Button nicht gefunden');

fromEvent(submitButton, 'click').pipe(
  exhaustMap(() => {
    const formData: FormData = {
      username: 'testuser',
      email: 'test@example.com'
    };
    // Promise-Funktion in Observable umwandeln
    return from(submitForm(formData));
  }),
  catchError(error => {
    console.error('Übermittlungsfehler:', error);
    return of({ success: false });
  })
).subscribe(result => {
  if (result.success) {
    console.log('Übermittlung erfolgreich');
  } else {
    console.log('Übermittlung fehlgeschlagen');
  }
});
```

Jedes Mal, wenn der Formular-Senden-Button geklickt wird, wird ein neuer Übermittlungsprozess gestartet, aber **während der Übermittlung werden neue Übermittlungen ignoriert**.

In diesem Beispiel verhindert die Verwendung von `exhaustMap` doppelte Anfragen während der Übermittlung.

### Such-Autovervollständigung

Beispiel zur Überwachung von Wertänderungen in einem Eingabeformular und Durchführung einer API-Suche.

```ts
import { fromEvent, from } from 'rxjs';
import { debounceTime, switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface SearchResult {
  items: Array<{
    login: string;
    id: number;
    avatar_url: string;
  }>;
  total_count: number;
}

// Promise-basierte API-Funktion
async function searchAPI(query: string): Promise<SearchResult> {
  const response = await fetch(`https://api.github.com/search/users?q=${query}`);
  if (!response.ok) {
    throw new Error('Suche fehlgeschlagen');
  }
  return response.json();
}

// Event-Stream mit RxJS verwalten
const label = document.createElement('label');
label.innerText = 'Suche: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);
if (!searchInput) throw new Error('Sucheingabefeld nicht gefunden');

fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  switchMap(event => {
    const query = (event.target as HTMLInputElement).value;
    // Promise-Funktion in Observable umwandeln
    return from(searchAPI(query));
  }),
  catchError(error => {
    console.error(error);
    return of({ items: [], total_count: 0 }); // Bei Fehler leeres Ergebnis zurückgeben
  })
).subscribe(result => {
  console.log('Suchergebnisse:', result.items);
  console.log('Gesamt:', result.total_count);
});
```

> [!TIP]
> **Design durch Trennung der Verantwortlichkeiten**
>
> - **RxJS**: Zuständig für Ereignissteuerung (debounce, switchMap etc.)
> - **Promise**: Zuständig für HTTP-Anfragen (async/await)
> - **`from()`**: Überbrückt beide
>
> Durch Verwendung jeder Technologie am richtigen Ort verbessern sich Lesbarkeit und Wartbarkeit des Codes.

## Vor- und Nachteile

### Promise
<div class="comparison-cards">

::: tip Vorteile
- Keine Abhängigkeiten erforderlich, da JavaScript-Standard
- Intuitiver und lesbarer Code mit `async/await`
- Niedrige Lernkosten
- Einfache Verarbeitung einzelner Aufgaben
:::

::: danger Nachteile
- Kann nicht mit mehreren Werten umgehen
- Keine Abbruchfunktion
- Ungeeignet für kontinuierliche Stream-Verarbeitung
- Schwierige komplexe Ereignisverarbeitung
:::

</div>

### RxJS
<div class="comparison-cards">

::: tip Vorteile
- Kann mehrere Werte zeitlich handhaben
- Komplexe Verarbeitungen mit umfangreichen Operatoren möglich
- Einfacher Abbruch mit `unsubscribe`
- Flexible Implementierung von Fehlerbehandlung und Wiederholung möglich
- Deklarativ und leicht testbar
:::

::: danger Nachteile
- Hohe Lernkosten
- Abhängigkeit von Bibliothek erforderlich
- Overhead vorhanden (übertrieben für kleine Projekte)
- Debugging kann schwierig sein
:::

</div>

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>

## Bereiche, in denen RxJS besonders glänzt

RxJS ist in den folgenden Bereichen besonders leistungsstark. Komplexe Anforderungen, die mit Promise allein schwer zu realisieren sind, können elegant gelöst werden.

| Bereich | Konkrete Beispiele | Vergleich mit Promise |
|------|--------|----------------|
| **Echtzeit-Kommunikation** | WebSocket, SSE, Chat, Aktienkursaktualisierung | Promise nur für einmalige Kommunikation. Ungeeignet für kontinuierliche Nachrichtenverarbeitung |
| **Benutzereingabesteuerung** | Such-Autovervollständigung, Formularvalidierung | debounce, distinctUntilChanged etc. standardmäßig ausgestattet |
| **Kombination mehrerer Quellen** | Kombination von Suchkriterien × Sortierreihenfolge × Filter | Prägnante Beschreibung mit combineLatest, withLatestFrom |
| **Offline-Unterstützung** | PWA, Netzwerkstatusüberwachung, automatische Neusynchronisation | Flexible Wiederholungssteuerung mit retry, retryWhen |
| **Streaming-API** | OpenAI, schrittweise Token-Ausgabe von KI-Antworten | Kontinuierliche Datenverarbeitung in Echtzeit möglich |
| **Abbruchsteuerung** | Unterbrechung lang laufender Verarbeitungen, Verwerfen alter Anfragen | Sofortiger Abbruch mit unsubscribe() möglich |

> [!NOTE]
> Details zu RxJS-Anwendungsbereichen finden Sie auch unter [Was ist RxJS - Anwendungsfälle](./what-is-rxjs.md#anwendungsfalle).

## Zusammenfassung

| Zweck | Empfohlen | Grund |
|------|------|------|
| Einzelne HTTP-Anfrage | Promise (`async/await`) | Einfach, lesbar, Standard-API |
| Verarbeitung von Benutzereingabeereignissen | RxJS | Steuerung wie debounce, distinct erforderlich |
| Echtzeitdaten (WebSocket) | RxJS | Kontinuierliche Nachrichten natürlich handhabbar |
| Parallele Ausführung mehrerer asynchroner Verarbeitungen | Promise (`Promise.all`) | Promise ausreichend für einfache parallele Ausführung |
| Kontinuierliche Event-Streams | RxJS | Kann mehrere Werte zeitlich handhaben |
| Abbrechbare Verarbeitung | RxJS | Zuverlässiger Abbruch mit unsubscribe() |
| Einfache Anwendung | Promise | Niedrige Lernkosten, wenige Abhängigkeiten |
| Angular-Anwendung | RxJS | Standardmäßig im Framework integriert |

### Grundprinzip
- **Wenn es einfach geht, Promise verwenden**
- **Wenn komplexe Stream-Verarbeitung erforderlich ist, RxJS verwenden**
- **Kombination beider ist auch effektiv** (Überbrückung mit `from()`)

RxJS ist leistungsstark, aber es ist nicht notwendig, RxJS für alle asynchronen Verarbeitungen zu verwenden. Es ist wichtig, das richtige Werkzeug in der richtigen Situation zu verwenden.

> [!TIP] Nächste Schritte
> - Lernen Sie die Details von Observable unter [Was ist Observable](/de/guide/observables/what-is-observable)
> - Lernen Sie die Erstellung von Observables unter [Creation Functions](/de/guide/creation-functions/index)
> - Lernen Sie Transformation und Steuerung von Observables unter [Operators](/de/guide/operators/index)
