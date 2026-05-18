---
description: "Der windowTime-Operator teilt Observable in Zeitintervallen. Verarbeitung als separate Observables, Batch-Verarbeitung und Unterschiede zu bufferTime."
---

# windowTime - Observable in festen Zeitintervallen teilen

Der `windowTime`-Operator gruppiert die Werte eines Quell-Observables **in festen Zeitintervallen** und gibt diese Gruppen als **neue Observables** aus.
Während `bufferTime` Arrays zurückgibt, gibt `windowTime` **Observable\<T> zurück**, sodass auf jedes Fenster weitere Operatoren angewendet werden können.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, take } from 'rxjs';

// Gibt alle 100ms einen Wert aus
const source$ = interval(100);

source$.pipe(
  windowTime(1000), // Erstellt alle 1 Sekunde ein Fenster
  take(3),          // Nur die ersten 3 Fenster
  mergeAll()        // Flacht jedes Fenster ab
).subscribe(value => {
  console.log('Wert:', value);
});

// Ausgabe:
// 1. Sekunde: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// 2. Sekunde: 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
// 3. Sekunde: 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
```

- In der angegebenen Zeit (1000ms) wird ein neues Fenster (Observable) erstellt.
- Jedes Fenster kann als unabhängiges Observable verarbeitet werden.

[🌐 Offizielle RxJS-Dokumentation - `windowTime`](https://rxjs.dev/api/operators/windowTime)

## 💡 Typische Anwendungsfälle

- **Zeitbasierte Batch-Verarbeitung**: Daten in regelmäßigen Zeitabständen gebündelt verarbeiten
- **Echtzeit-Datenaggreggation**: Anzahl der Ereignisse pro Sekunde zählen
- **Performance-Überwachung**: Metriken in regelmäßigen Zeitabständen sammeln
- **Zeitreihenanalyse**: Statistische Verarbeitung nach Zeitfenstern

## 🔍 Unterschied zu bufferTime

| Operator | Ausgabe | Anwendungsfall |
|:---|:---|:---|
| `bufferTime` | **Array (T[])** | Gruppierte Werte gemeinsam verarbeiten |
| `windowTime` | **Observable\<T>** | Unterschiedliche Stream-Verarbeitung pro Zeitfenster |

```ts
import { interval } from 'rxjs';
import { bufferTime, windowTime, take } from 'rxjs';

const source$ = interval(100);

// bufferTime - Ausgabe als Array
source$.pipe(
  bufferTime(1000),
  take(2)
).subscribe(values => {
  console.log('Buffer (Array):', values);
  // Ausgabe: Buffer (Array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// windowTime - Ausgabe als Observable
source$.pipe(
  windowTime(1000),
  take(2)
).subscribe(window$ => {
  console.log('Fenster (Observable):', window$);
  // Verschachtelte Subscription: erforderliches Pattern der window-Operatoren-Spezifikation
  // (notwendig zum Konsumieren der inneren Observable, die von windowTime emittiert wird)
  window$.subscribe(value => {
    console.log('  Wert:', value);
  });
});
```

## 🧠 Praxis-Codebeispiel 1: Klicks pro Sekunde zählen

Ein Beispiel, das die Anzahl der Button-Klicks jede Sekunde aggregiert.

```ts
import { fromEvent } from 'rxjs';
import { windowTime, map, mergeAll, scan } from 'rxjs';

// Button erstellen
const button = document.createElement('button');
button.textContent = 'Klicken';
document.body.appendChild(button);

// Ausgabebereich
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Klick-Ereignis
const clicks$ = fromEvent(button, 'click');

let windowNumber = 0;

clicks$.pipe(
  windowTime(1000), // Erstellt alle 1 Sekunde ein Fenster
  map(window$ => {
    ++windowNumber;

    // Zählt Klicks innerhalb jedes Fensters
    return window$.pipe(
      scan(count => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  const timestamp = new Date().toLocaleTimeString();
  output.textContent = `[${timestamp}] Fenster ${windowNumber}: ${count} Klick(s)`;
});
```

- Alle 1 Sekunde wird ein neues Fenster erstellt.
- Die Anzahl der Klicks innerhalb jedes Fensters wird in Echtzeit gezählt.

## 🎯 Praxis-Codebeispiel 2: Statistische Verarbeitung pro Zeitfenster

Ein Beispiel, das Summe und Durchschnitt der Werte in jedem Zeitfenster berechnet.

```ts
import { interval } from 'rxjs';
import { windowTime, map, mergeMap, toArray, take } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>Statistische Verarbeitung pro Zeitfenster (alle 1 Sekunde)</h3>';
document.body.appendChild(output);

const table = document.createElement('table');
table.style.borderCollapse = 'collapse';
table.style.marginTop = '10px';
table.innerHTML = `
  <thead>
    <tr style="background: #f0f0f0;">
      <th style="border: 1px solid #ccc; padding: 8px;">Fenster</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Anzahl</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Summe</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Durchschnitt</th>
    </tr>
  </thead>
  <tbody id="stats-body"></tbody>
`;
output.appendChild(table);

const source$ = interval(100).pipe(
  map(() => Math.floor(Math.random() * 100)) // Zufälliger Wert
);

let windowNumber = 0;

source$.pipe(
  windowTime(1000), // Alle 1 Sekunde
  take(5),          // Nur 5 Fenster
  mergeMap(window$ => {
    const current = ++windowNumber;

    // Werte jedes Fensters in Array umwandeln und statistisch verarbeiten
    return window$.pipe(
      toArray(),
      map(values => ({
        window: current,
        count: values.length,
        sum: values.reduce((a, b) => a + b, 0),
        avg: values.length > 0
          ? (values.reduce((a, b) => a + b, 0) / values.length).toFixed(2)
          : 0
      }))
    );
  })
).subscribe(stats => {
  const tbody = document.getElementById('stats-body')!;
  const row = document.createElement('tr');
  row.innerHTML = `
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.window}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.count}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.sum}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.avg}</td>
  `;
  tbody.appendChild(row);
});
```

- Die statistischen Informationen jedes Fensters können individuell berechnet werden.
- Verschiedene Verarbeitungen können für jedes Fenster angewendet werden.
- Statistische Daten werden visuell im Tabellenformat angezeigt.

## 📊 Überlappende Fenster (windowCreationInterval)

Wenn Sie das zweite Argument `windowCreationInterval` angeben, können Sie Fenster überlappen lassen.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray, take, map } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>Überlappende Fenster</h3>';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.marginTop = '10px';
document.body.appendChild(output);

const source$ = interval(100);

source$.pipe(
  windowTime(
    2000,  // Fensterlänge: 2 Sekunden
    1000   // Fenster-Erstellungsintervall: 1 Sekunde
  ),
  take(3),
  mergeMap((window$, index) =>
    window$.pipe(
      toArray(),
      map(values => ({ window: index + 1, values }))
    )
  )
).subscribe(result => {
  const div = document.createElement('div');
  div.style.marginTop = '10px';
  div.style.padding = '5px';
  div.style.backgroundColor = '#f5f5f5';
  div.style.borderLeft = '3px solid #4CAF50';

  const title = document.createElement('strong');
  title.textContent = `Fenster ${result.window}:`;
  div.appendChild(title);

  div.appendChild(document.createElement('br'));

  const values = document.createElement('span');
  values.textContent = `Werte: [${result.values.join(', ')}]`;
  div.appendChild(values);

  div.appendChild(document.createElement('br'));

  const info = document.createElement('span');
  info.style.color = '#666';
  info.textContent = `(${result.values.length} Werte, ${(result.window - 1)} Sek. ~ ${(result.window + 1)} Sek.)`;
  div.appendChild(info);

  output.appendChild(div);

  // Chrome-Workaround: Erzwingt Rendering
  void output.offsetHeight;
});
```

**Erklärung der Funktionsweise:**
- **Fenster 1**: Werte von 0-2 Sek. `[0, 1, 2, ..., 19]` (20 Stück)
- **Fenster 2**: Werte von 1-3 Sek. `[10, 11, 12, ..., 29]` (20 Stück) ← Werte 10-19 überlappen mit Fenster 1
- **Fenster 3**: Werte von 2-4 Sek. `[20, 21, 22, ..., 39]` (20 Stück) ← Werte 20-29 überlappen mit Fenster 2

- Wenn neue Fenster in kürzeren Abständen (1 Sekunde) als die Fensterlänge (2 Sekunden) erstellt werden, entstehen Überlappungen.
- Nützlich für die Implementierung von Sliding-Windows.

## 🎯 Praktisches Beispiel: Echtzeit-Ereignisüberwachung

```ts
import { fromEvent } from 'rxjs';
import { windowTime, mergeMap, toArray, map } from 'rxjs';

// Ausgabebereich
const output = document.createElement('div');
output.innerHTML = '<h3>Mausbewegungsüberwachung (alle 5 Sekunden)</h3>';
document.body.appendChild(output);

const list = document.createElement('ul');
output.appendChild(list);

// Mausbewegungsereignis
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  windowTime(5000), // Alle 5 Sekunden
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(events => ({
        count: events.length,
        timestamp: new Date().toLocaleTimeString()
      }))
    )
  )
).subscribe(result => {
  const item = document.createElement('li');
  item.textContent = `[${result.timestamp}] Mausbewegungen: ${result.count}mal`;
  list.insertBefore(item, list.firstChild);

  // Maximal 10 Einträge anzeigen
  while (list.children.length > 10) {
    list.removeChild(list.lastChild!);
  }
});
```

## ⚠️ Wichtige Hinweise

### 1. Verwaltung von Fenster-Subscriptions

Da jedes Fenster ein unabhängiges Observable ist, muss es explizit abonniert werden.

```ts
source$.pipe(
  windowTime(1000)
).subscribe(window$ => {
  // Verschachtelte Subscription: erforderliches Pattern der window-Operatoren-Spezifikation
  // Werte fließen nicht, wenn das Fenster selbst nicht abonniert wird
  window$.subscribe(value => {
    console.log('Wert:', value);
  });
});
```

Oder verwenden Sie `mergeAll()`, `concatAll()`, `switchAll()` usw., um zu flatten.

```ts
source$.pipe(
  windowTime(1000),
  mergeAll() // Alle Fenster zusammenführen
).subscribe(value => {
  console.log('Wert:', value);
});
```

### 2. Speicherverwaltung

Bei langfristiger Ausführung ist es wichtig, ordnungsgemäß zu deabonnieren.

```ts
import { takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

const destroy$ = new Subject<void>();

source$.pipe(
  windowTime(1000),
  mergeAll(),
  takeUntil(destroy$) // Bei Zerstörung deabonnieren
).subscribe();

// Z.B. beim Zerstören von Komponenten
destroy$.next();
destroy$.complete();
```

### 3. Maximale Werteanzahl angeben (maxWindowSize)

Mit dem dritten Argument können Sie die maximale Anzahl der Werte pro Fenster begrenzen.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray } from 'rxjs';

interval(100).pipe(
  windowTime(
    2000,      // Fensterlänge: 2 Sekunden
    undefined, // Fenster-Erstellungsintervall: Standard (keine Überlappung)
    5          // Maximale Werteanzahl: bis zu 5 Werte
  ),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Fenster:', values);
  // Enthält maximal 5 Werte
});
```

## 🆚 Vergleich der window-Operatoren

| Operator | Trennungszeitpunkt | Anwendungsfall |
|:---|:---|:---|
| `window` | Emission eines anderen Observables | Ereignisgesteuerte Teilung |
| `windowTime` | **Festes Zeitintervall** | **Zeitbasierte Teilung** |
| `windowCount` | Feste Anzahl | Anzahlbasierte Teilung |
| `windowToggle` | Start- und End-Observable | Dynamische Start-/Ende-Steuerung |
| `windowWhen` | Dynamische Schließbedingung | Unterschiedliche Endbedingungen pro Fenster |

## 📚 Verwandte Operatoren

- **[bufferTime](./bufferTime)** - Werte als Array sammeln (Array-Version von windowTime)
- **[window](./window)** - Fenster-Teilung durch Observable-Emission
- **[windowCount](./windowCount)** - Fenster-Teilung basierend auf Anzahl
- **[windowToggle](./windowToggle)** - Fenstersteuerung mit Start- und End-Observables
- **[windowWhen](./windowWhen)** - Fenster-Teilung mit dynamischer Schließbedingung

## Zusammenfassung

Der `windowTime`-Operator ist ein leistungsstarkes Tool, das Streams zeitbasiert teilt und jedes Zeitfenster als unabhängiges Observable verarbeiten kann.

- ✅ Erstellt automatisch Fenster in festen Zeitintervallen
- ✅ Verschiedene Verarbeitungen können auf jedes Fenster angewendet werden
- ✅ Unterstützt auch Sliding-Windows (Überlappung)
- ✅ Ideal für Aggregation und Analyse von Echtzeitdaten
- ⚠️ Subscription-Verwaltung erforderlich
- ⚠️ Achten Sie auf die Speicherverwaltung
