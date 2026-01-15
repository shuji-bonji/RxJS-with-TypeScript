---
description: "Der pairwise-Operator gibt zwei aufeinanderfolgende Werte als Paar-Array [vorheriger Wert, aktueller Wert] aus. Kann für Wertänderungserkennung, Differenzberechnung, Trendanalyse, Animationsinterpolation und andere Vergleichsverarbeitungen zwischen vorherigen und aktuellen Werten verwendet werden. Erklärt typsichere Implementierung mit TypeScript und praktische Beispiele."
---

# pairwise - Aufeinanderfolgende Paare

Der `pairwise`-Operator fasst **zwei aufeinanderfolgende Werte aus dem Stream als Array `[vorheriger Wert, aktueller Wert]`** zusammen und gibt sie aus.
Praktisch zum Vergleichen von vorherigen und aktuellen Werten oder zum Berechnen von Änderungsbeträgen.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { pairwise, take } from 'rxjs';

interval(1000).pipe(
  take(6),
  pairwise()
).subscribe(console.log);

// Ausgabe:
// [0, 1]
// [1, 2]
// [2, 3]
// [3, 4]
// [4, 5]
```

- Der erste Wert (0) wird nicht einzeln ausgegeben, sondern erst wenn der zweite Wert (1) kommt als `[0, 1]`.
- Es wird immer **das Paar aus vorherigem Wert und aktuellem Wert** ausgegeben.

[🌐 RxJS Offizielle Dokumentation - `pairwise`](https://rxjs.dev/api/operators/pairwise)

## 💡 Typische Anwendungsmuster

- Berechnung von Maus- oder Touch-Bewegungen
- Berechnung von Preis- oder Zahlenänderungen (Differenzen)
- Zustandsänderungserkennung (Vergleich von vorherigem und aktuellem Zustand)
- Bestimmung der Scroll-Richtung

## 🧠 Praktisches Codebeispiel (mit UI)

Beispiel zur Anzeige von Mausbewegungsrichtung und -betrag.

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Mausbewegungsereignis
fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY })),
  pairwise()
).subscribe(([prev, curr]) => {
  const deltaX = curr.x - prev.x;
  const deltaY = curr.y - prev.y;
  const direction = deltaX > 0 ? 'rechts' : deltaX < 0 ? 'links' : 'Stopp';

  output.innerHTML = `
    Vorher: (${prev.x}, ${prev.y})<br>
    Jetzt: (${curr.x}, ${curr.y})<br>
    Bewegung: Δx=${deltaX}, Δy=${deltaY}<br>
    Richtung: ${direction}
  `;
});
```

- Bei Mausbewegung werden vorherige und aktuelle Koordinaten sowie Bewegungsbetrag angezeigt.
- Mit `pairwise` werden vorherige und aktuelle Koordinaten automatisch als Paar abgerufen.

## 🎯 Beispiel zur Berechnung numerischer Änderungen

Praktisches Beispiel zur Berechnung von Änderungen (Differenzen) in einem numerischen Stream.

```ts
import { interval } from 'rxjs';
import { map, pairwise, take } from 'rxjs';

// 0, 1, 4, 9, 16, 25 (Quadratzahlen)
interval(500).pipe(
  take(6),
  map(n => n * n),
  pairwise(),
  map(([prev, curr]) => ({
    prev,
    curr,
    diff: curr - prev
  }))
).subscribe(result => {
  console.log(`${result.prev} → ${result.curr} (Differenz: +${result.diff})`);
});

// Ausgabe:
// 0 → 1 (Differenz: +1)
// 1 → 4 (Differenz: +3)
// 4 → 9 (Differenz: +5)
// 9 → 16 (Differenz: +7)
// 16 → 25 (Differenz: +9)
```

## 🎯 Bestimmung der Scroll-Richtung

Beispiel zur Bestimmung der Scroll-Richtung (auf/ab).

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise, throttleTime } from 'rxjs';

// Fest angezeigten Ausgabebereich erstellen
const output = document.createElement('div');
output.style.position = 'fixed';
output.style.top = '10px';
output.style.right = '10px';
output.style.padding = '15px';
output.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
output.style.color = 'white';
output.style.fontFamily = 'monospace';
output.style.fontSize = '14px';
output.style.borderRadius = '5px';
output.style.zIndex = '9999';
document.body.appendChild(output);

// Dummy-Inhalt für scrollbare Seite
const content = document.createElement('div');
content.style.height = '200vh'; // Seitenhöhe verdoppeln
content.innerHTML = '<h1>Bitte nach unten scrollen</h1>';
document.body.appendChild(content);

// Scroll-Position abrufen
fromEvent(window, 'scroll').pipe(
  throttleTime(100), // Ausdünnung alle 100ms
  map(() => window.scrollY),
  pairwise()
).subscribe(([prevY, currY]) => {
  const diff = currY - prevY;
  const direction = diff > 0 ? '↓ unten' : '↑ oben';
  const arrow = diff > 0 ? '⬇️' : '⬆️';

  output.innerHTML = `
    ${arrow} Scroll-Richtung: ${direction}<br>
    Vorherige Position: ${prevY.toFixed(0)}px<br>
    Aktuelle Position: ${currY.toFixed(0)}px<br>
    Bewegungsbetrag: ${Math.abs(diff).toFixed(0)}px
  `;
});
```

- Beim Scrollen der Seite werden Richtung und Positionsinformationen im fest angezeigten Bereich oben rechts angezeigt.
- Mit `pairwise` werden vorherige und aktuelle Scroll-Position automatisch als Paar abgerufen.

## 🎯 Typsichere Nutzung von pairwise

Beispiel mit TypeScript-Typinferenz.

```ts
import { from } from 'rxjs';
import { pairwise } from 'rxjs';

interface Stock {
  symbol: string;
  price: number;
  timestamp: number;
}

const stockPrices: Stock[] = [
  { symbol: 'AAPL', price: 150, timestamp: 1000 },
  { symbol: 'AAPL', price: 152, timestamp: 2000 },
  { symbol: 'AAPL', price: 148, timestamp: 3000 },
  { symbol: 'AAPL', price: 155, timestamp: 4000 },
];

from(stockPrices).pipe(
  pairwise()
).subscribe(([prev, curr]) => {
  const change = curr.price - prev.price;
  const changePercent = ((change / prev.price) * 100).toFixed(2);
  const trend = change > 0 ? '📈' : change < 0 ? '📉' : '➡️';

  console.log(
    `${curr.symbol}: $${prev.price} → $${curr.price} ` +
    `(${changePercent}%) ${trend}`
  );
});

// Ausgabe:
// AAPL: $150 → $152 (1.33%) 📈
// AAPL: $152 → $148 (-2.63%) 📉
// AAPL: $148 → $155 (4.73%) 📈
```

## 🔍 Vergleich mit bufferCount(2, 1)

`pairwise()` verhält sich identisch zu `bufferCount(2, 1)`.

```ts
import { of } from 'rxjs';
import { pairwise, bufferCount } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== pairwise ===');
source$.pipe(pairwise()).subscribe(console.log);
// Ausgabe: [1,2], [2,3], [3,4], [4,5]

console.log('=== bufferCount(2, 1) ===');
source$.pipe(bufferCount(2, 1)).subscribe(console.log);
// Ausgabe: [1,2], [2,3], [3,4], [4,5]
```

**Unterscheidung**:
- `pairwise()`: Explizit für Paare aus zwei aufeinanderfolgenden Werten, Code-Intention ist klar
- `bufferCount(2, 1)`: Flexibler (kann auch Fenstergrößen von 3 oder mehr unterstützen)

## ⚠️ Achtung

### Erster Wert wird nicht ausgegeben

`pairwise` gibt nichts aus, bis zwei Werte vorhanden sind, daher kann der erste Wert nicht einzeln abgerufen werden.

```ts
import { of } from 'rxjs';
import { pairwise } from 'rxjs';

of(1).pipe(pairwise()).subscribe({
  next: console.log,
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Abgeschlossen
// (Kein Wert wird ausgegeben)
```

**Gegenmaßnahme**: Wenn der erste Wert auch verarbeitet werden soll, fügen Sie mit `startWith` einen Anfangswert hinzu.

```ts
import { of } from 'rxjs';
import { startWith, pairwise } from 'rxjs';

of(10, 20, 30).pipe(
  startWith(0),
  pairwise()
).subscribe(console.log);

// Ausgabe:
// [0, 10]
// [10, 20]
// [20, 30]
```

### Speichernutzung

`pairwise` hält nur den unmittelbar vorherigen Wert, daher ist die Speichereffizienz gut.

## 📚 Verwandte Operatoren

- [`scan`](./scan) - Komplexere kumulative Verarbeitung
- [`bufferCount`](./bufferCount) - Werte in angegebener Anzahl zusammenfassen
- [`distinctUntilChanged`](../filtering/distinctUntilChanged) - Aufeinanderfolgende doppelte Werte entfernen
- [`startWith`](../utility/startWith) - Anfangswert hinzufügen

## Zusammenfassung

Der `pairwise`-Operator gibt zwei aufeinanderfolgende Werte als Paar `[vorheriger Wert, aktueller Wert]` aus. Sehr praktisch für **Situationen, in denen Vergleiche zwischen vorherigem und aktuellem Wert erforderlich sind**, wie Mausbewegungsverfolgung, Preisschwankungsberechnungen oder Zustandsübergangserkennung. Zu beachten ist, dass der erste Wert erst ausgegeben wird, wenn der zweite Wert eintrifft, aber dies kann durch Hinzufügen eines Anfangswerts mit `startWith` gelöst werden.
