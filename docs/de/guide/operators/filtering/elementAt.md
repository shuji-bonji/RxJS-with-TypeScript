---
description: "Der elementAt-Operator ist ein RxJS-Filterungsoperator, der nur Werte an einer bestimmten Indexposition abruft. Er funktioniert ähnlich wie der Array-Index-Zugriff."
---

# elementAt - Abruf durch Indexangabe

Der Operator "elementAt" ruft **nur den Wert an der angegebenen Indexposition** aus dem Observable ab und schließt den Stream sofort ab. Er funktioniert ähnlich wie `array[index]` eines Arrays.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Ausgabe.: 30(Index2Wert)
```

**Ablauf der Operation**:.
1. 10 (Index 0) → Überspringen
2. 20 (Index 1) → Überspringen
3. 30 (Index 2) → Ausgabe und Abschluss
4. 40, 50 nicht ausgewertet

[🌐 Offizielle RxJS-Dokumentation - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Typisches Nutzungsmuster.

- **Pagination**: das erste Element auf einer bestimmten Seite holen.
- **Beschaffung von Daten mit Ordnungsgarantie**: Abrufen des N-ten Ereignisses oder der N-ten Nachricht.
- **Testen und Debuggen**: Überprüfen des Wertes einer bestimmten Position.
- **Array-ähnlicher Zugriff**: Observable wie ein Array behandeln

## 🧠 Praktisches Codebeispiel 1: Countdown von Ereignissen

Dies ist ein Beispiel für das Ausführen einer Aktion beim N-ten Klick.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UIerstellen
const output = document.createElement('div');
output.innerHTML = '<h3>5Klicken Sie einmal, um die Meldung anzuzeigen</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Klicken Sie auf';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'mehr5Einmal klicken';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Ereignis anklicken
const clicks$ = fromEvent(button, 'click');

// Für Zählanzeige
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `mehr${remaining}Einmal klicken`;
  } else {
    counter.textContent = '';
  }
});

// 5Zweite Zeit (Index)4Erkannte Klicks von
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Erreichte！';
  result.style.color = 'green';
  button.disabled = true;
});
```

- Der fünfte Klick (Index 4) schließt die Aktion ab.
- Er beginnt bei 0, genau wie der Array-Index.

## 🎯 Praktisches Codebeispiel 2: Die N-te Zahl aus dem Datenstrom holen.

Dies ist ein Beispiel für das Abrufen einer bestimmten Reihenfolge von Werten aus Daten, die in regelmäßigen Abständen veröffentlicht werden.

```ts
import { interval } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UIerstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Aus dem DatenstromNHolen Sie die zweite';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Geben Sie den Index (0〜aus dem Datenstrom (9)';
input.min = '0';
input.max = '9';
input.style.marginRight = '10px';
container.appendChild(input);

const getButton = document.createElement('button');
getButton.textContent = 'Abrufen von';
container.appendChild(getButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Datenstrom (0.5Die Werte werden jede Sekunde ausgegeben,10bis zu 1)
const data$ = interval(500).pipe(
  map(i => ({ index: i, value: Math.floor(Math.random() * 100), timestamp: Date.now() }))
);

getButton.addEventListener('click', () => {
  const index = parseInt(input.value);

  if (isNaN(index) || index < 0 || index > 9) {
    status.textContent = '0〜aus dem Datenstrom (9Bitte geben Sie einen Bereich von';
    status.style.color = 'red';
    return;
  }

  status.textContent = `Index ${index} Wert wird abgerufen...`;
  status.style.color = 'blue';
  result.style.display = 'none';
  getButton.disabled = true;
  input.disabled = true;

  data$.pipe(
    elementAt(index)
  ).subscribe({
    next: data => {
      status.textContent = '';
      result.style.display = 'block';
      result.innerHTML = `
        <strong>✅ Erfolgreicher Abruf</strong><br>
        Index: ${data.index}<br>
        Wert: ${data.value}<br>
        Zeitstempel: ${new Date(data.timestamp).toLocaleTimeString()}
      `;
      result.style.color = 'green';
      result.style.backgroundColor = '#e8f5e9';
      getButton.disabled = false;
      input.disabled = false;
    },
    error: err => {
      status.textContent = '';
      result.style.display = 'block';
      result.textContent = `❌ Fehler: ${err.message}`;
      result.style.color = 'red';
      result.style.backgroundColor = '#ffebee';
      getButton.disabled = false;
      input.disabled = false;
    }
  });
});
```

- Ruft Werte mit einem bestimmten Index aus einem Datenstrom ab, der alle 0,5 Sekunden ausgegeben wird.
- Ein Fehler wird erzeugt, wenn der Index außerhalb des Bereichs liegt.

## 🆚 Vergleich mit ähnlichen Operatoren

### elementAt vs take vs first

```ts
import { from } from 'rxjs';
import { elementAt, take, first, skip } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// elementAt: Es werden nur Werte mit einem bestimmten Index abgerufen
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Ausgabe.: 30

// take: Vom Anfang anNAbrufen eines Wertes vom Anfang an
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Ausgabe.: 10, 20, 30

// skip + first: elementAt Äquivalent zu (redundant)
numbers$.pipe(
  skip(2),
  first()
).subscribe(console.log);
// Ausgabe.: 30
```

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Ausgabe.: 30(Index2Wert)
```

## ⚠️ Anmerkungen.

### 1. wenn der Index außerhalb des Bereichs liegt

Wenn der angegebene Index nicht erreicht wird, bevor der Stream abgeschlossen ist, wird ein Fehler erzeugt.


```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // 3Nur einen

numbers$.pipe(
  elementAt(5) // Index5anfordern
).subscribe({
  next: console.log,
  error: err => console.error('Fehler:', err.message)
});
// Ausgabe.: Fehler: no elements in sequence
```

### 2. geben Sie Standardwerte an.

Um Fehler zu vermeiden, können Standardwerte angegeben werden.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Geben Sie einen Standardwert an
numbers$.pipe(
  elementAt(5, 999) // Index5Wenn nicht vorhanden, wird zurückgegeben999Gibt einen
).subscribe({
  next: console.log,
  error: err => console.error('Fehler:', err.message)
});
// Ausgabe.: 999
```

### 3. mit asynchronen Streams verwenden

Bei asynchronen Streams wird gewartet, bis die Indexposition erreicht ist.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// 1Gibt jede Sekunde einen Wert aus
interval(1000).pipe(
  elementAt(3) // Index3(4(Sekundenwert)
).subscribe(console.log);
// 3Ausgabe nach Sekunden: 3
```

### 4. negative Indizes sind nicht erlaubt

Negative Indizes können nicht angegeben werden.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ❌ Negative Indizes sind Fehler
numbers$.pipe(
  elementAt(-1)
).subscribe({
  next: console.log,
  error: err => console.error('Fehler:', err.message)
});
// Fehler: ArgumentOutOfRangeError: index out of range
```

Verwenden Sie `takeLast` oder `last`, um vom Ende des Arrays auszugehen.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Letzten Wert abfragen
numbers$.pipe(
  last()
).subscribe(console.log);
// Ausgabe.: 50

// ✅ Ermittelt den letztenNErmittelt den letzten Wert
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Ausgabe.: 40, 50
```

## 📚 Verwandte Operatoren.

- **[take](. /take)** - N wird von Anfang an genommen.
- **[first](. /first)** - liefert den ersten Wert.
- **[last](. /last)** - liefert den letzten Wert
- **[skip](. /skip)** - überspringt die ersten N Werte
- **[takeLast](. /takeLast)** - holt die letzten N Werte

## Zusammenfassung.

Der Operator "elementAt" ruft nur den Wert an der angegebenen Indexposition ab.

- ✅ Gleiches Verhalten wie beim Array-Index-Zugriff.
- ✅ Ideal zum Abrufen des N-ten Wertes.
- ✅ Es können Standardwerte angegeben werden, um Fehler zu vermeiden
- ⚠️ Fehler, wenn der Index außerhalb des Bereichs liegt (kein Standardwert)
- ⚠️ Negative Indizes sind nicht erlaubt
- ⚠️ Asynchrone Ströme warten, bis der Wert erreicht ist
