---
description: "Der buffer-Operator gibt akkumulierte Werte als Array aus, wenn ein anderes Observable einen Wert emittiert. Ideal für ereignisgesteuerte Batch-Verarbeitung wie Massen-Submit per Button-Klick oder Datenspeicherung beim Schließen eines Fensters. Erklärt typsichere Implementierung mit TypeScript."
---

# buffer - Werte zum Zeitpunkt eines anderen Observables zusammenfassen

Der `buffer`-Operator sammelt Werte des Quell-Observables, **bis ein anderes Observable einen Wert emittiert**, und gibt dann die gesammelten Werte **als Array auf einmal** aus.
Praktisch, wenn Sie die Pufferung nicht nach Zeit oder Anzahl, sondern nach externen Ereignissen oder Signalen steuern möchten.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval, fromEvent } from 'rxjs';
import { buffer } from 'rxjs';

// Werte alle 100ms emittieren
const source$ = interval(100);

// Klick-Ereignis als Trigger verwenden
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  buffer(clicks$)
).subscribe(bufferedValues => {
  console.log('Bis zum Klick akkumulierte Werte:', bufferedValues);
});

// Ausgabebeispiel (bei jedem Klick):
// Bis zum Klick akkumulierte Werte: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
// Bis zum Klick akkumulierte Werte: [11, 12, 13, 14, 15, 16, 17]
// ...
```

- Jedes Mal, wenn `clicks$` einen Wert emittiert, werden die bis dahin gesammelten Werte als Array ausgegeben.
- Das Merkmal ist, dass die Pufferabgrenzung durch ein externes Observable gesteuert werden kann.

[🌐 RxJS Offizielle Dokumentation - `buffer`](https://rxjs.dev/api/operators/buffer)

## 💡 Typische Anwendungsmuster

- Batch-Verarbeitung mit Benutzeraktion als Trigger
- Datensammlung und -übertragung basierend auf externen Signalen
- Ereignisgruppierung mit dynamischer Abgrenzung
- Gesammelte Übertragung bei WebSocket- oder API-Verbindungsaufbau

## 🔍 Unterschied zu bufferTime / bufferCount

| Operator | Zeitpunkt der Abgrenzung | Verwendungszweck |
|:---|:---|:---|
| `buffer` | **Emission eines anderen Observables** | Ereignisgesteuerte Kontrolle |
| `bufferTime` | **Feste Zeitspanne** | Zeitbasierte Batch-Verarbeitung |
| `bufferCount` | **Feste Anzahl** | Anzahlbasierte Batch-Verarbeitung |

```ts
import { interval, timer } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);
// Trigger alle 1 Sekunde
const trigger$ = timer(1000, 1000);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Werte jede Sekunde:', values);
});

// Ausgabe:
// Werte jede Sekunde: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Werte jede Sekunde: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
```

## 🧠 Praktisches Codebeispiel (mit UI)

Beispiel, das Mausbewegungsereignisse bis zum Button-Klick als Trigger aufzeichnet.

```ts
import { fromEvent } from 'rxjs';
import { map, buffer } from 'rxjs';

// Button und Ausgabebereich erstellen
const button = document.createElement('button');
button.textContent = 'Mausbewegung aufzeichnen';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Mausbewegungsereignis
const mouseMoves$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
);

// Button-Klick als Trigger
const clicks$ = fromEvent(button, 'click');

mouseMoves$.pipe(
  buffer(clicks$)
).subscribe(positions => {
  const message = `Erkannte Ereignisse: ${positions.length} Stück`;
  console.log(message);
  console.log('Koordinatendaten:', positions.slice(0, 5)); // Nur die ersten 5 anzeigen
  output.textContent = message;
});
```

- Alle Mausbewegungen bis zum Button-Klick werden im Puffer gesammelt.
- Da sie beim Klick zusammen verarbeitet werden, ist Batch-Verarbeitung zu einem beliebigen Zeitpunkt möglich.

## 🎯 Erweitertes Beispiel mit mehreren Triggern

Durch Kombination mehrerer Trigger-Observables ist flexiblere Steuerung möglich.

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { buffer, mapTo } from 'rxjs';

const source$ = interval(100);

// Mehrere Trigger: Klick oder 5 Sekunden vergangen
const clicks$ = fromEvent(document, 'click').pipe(mapTo('click'));
const fiveSeconds$ = timer(5000, 5000).pipe(mapTo('timer'));
const trigger$ = merge(clicks$, fiveSeconds$);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log(`Puffer-Ausgabe (${values.length} Stück):`, values);
});
```

## ⚠️ Achtung

### Vorsicht vor Memory Leaks

Da `buffer` Werte bis zum nächsten Trigger sammelt, kann es bei lange ausbleibendem Trigger zu Speicherdruck kommen.

```ts
// Schlechtes Beispiel: Trigger könnte nicht auftreten
const neverTrigger$ = fromEvent(document.querySelector('.non-existent'), 'click');

source$.pipe(
  buffer(neverTrigger$) // Trigger tritt nicht auf, Puffer sammelt unendlich
).subscribe();
```

**Gegenmaßnahmen**:
- Maximale Puffergröße durch Kombination mit `bufferTime` oder `bufferCount` begrenzen
- Timeout-Verarbeitung hinzufügen

```ts
import { interval, fromEvent, timer, race } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);

// Mehrere Trigger: Klick oder 5 Sekunden vergangen
const clicks$ = fromEvent(document, 'click');
const timeout$ = timer(10000); // Timeout nach maximal 10 Sekunden

source$.pipe(
  buffer(race(clicks$, timeout$)) // Whichever kommt zuerst
).subscribe(values => {
  console.log('Puffer:', values);
});
```

## 📚 Verwandte Operatoren

- [`bufferTime`](./bufferTime) - Zeitbasierte Pufferung
- [`bufferCount`](./bufferCount) - Anzahlbasierte Pufferung
- [`bufferToggle`](https://rxjs.dev/api/operators/bufferToggle) - Pufferungssteuerung mit Start- und End-Observable
- [`bufferWhen`](https://rxjs.dev/api/operators/bufferWhen) - Pufferung mit dynamischer Abschlussbedingung
- [`window`](./windowTime) - Gibt Observable statt Puffer zurück

## Zusammenfassung

Der `buffer`-Operator ist ein leistungsstarkes Werkzeug zum Zusammenfassen und Verarbeiten von Werten mit einem externen Observable als Trigger. Ermöglicht **ereignisgesteuerte** Batch-Verarbeitung statt zeit- oder anzahlbasiert. Allerdings ist Vorsicht geboten vor Memory Leaks, wenn der Trigger nicht auftritt.
