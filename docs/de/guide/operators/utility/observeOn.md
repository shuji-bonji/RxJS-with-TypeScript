---
description: Der observeOn-Operator steuert das Emissionstiming und den Ausführungskontext von Observable-Werten mit einem angegebenen Scheduler und wird zur Optimierung asynchroner Verarbeitung und Animationen genutzt.
---

# observeOn - Steuerung des Ausführungskontexts

Der `observeOn`-Operator steuert **das Emissionstiming und den Ausführungskontext von Observable-Werten mit einem angegebenen Scheduler**. Er ermöglicht die Ausführung nachfolgender Verarbeitungen auf einem bestimmten Scheduler.

## 🔰 Grundlegende Syntax und Funktionsweise

Durch Angabe eines Schedulers wird die nachfolgende Verarbeitung asynchron gemacht.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(v => console.log('Wert:', v));

console.log('Ende');

// Ausgabe:
// Start
// Ende
// Wert: 1
// Wert: 2
// Wert: 3
```

Verarbeitungen vor `observeOn` werden synchron ausgeführt, während Verarbeitungen nach `observeOn` mit dem angegebenen Scheduler ausgeführt werden.

[🌐 RxJS Offizielle Dokumentation - observeOn](https://rxjs.dev/api/index/function/observeOn)

## 💡 Typische Anwendungsfälle

- **Vermeidung von UI-Thread-Blockierung**: Schwere Verarbeitung asynchron machen
- **Animation-Optimierung**: Flüssiges Rendering mit `animationFrameScheduler`
- **Priorisierung von Verarbeitungen**: Steuerung des Ausführungstimings mit verschiedenen Schedulern
- **Mikrotask/Makrotask-Steuerung**: Feinabstimmung des Ausführungstimings

## Arten von Schedulern

| Scheduler | Merkmale | Anwendungsfall |
|:---|:---|:---|
| `asyncScheduler` | Basiert auf `setTimeout` | Allgemeine asynchrone Verarbeitung |
| `asapScheduler` | Mikrotask (Promise.then) | Schnellstmögliche asynchrone Ausführung |
| `queueScheduler` | Synchrone Warteschlange | Optimierung rekursiver Verarbeitung |
| `animationFrameScheduler` | `requestAnimationFrame` | Animation, 60fps-Rendering |

> [!TIP]
> Details zu Schedulern finden Sie unter [Scheduler-Typen und Verwendung](../../schedulers/types.md).

## 🧪 Praktisches Codebeispiel 1: Vermeidung von UI-Blockierung

Ein Beispiel für die asynchrone Ausführung großer Datenverarbeitung in Batches.

```ts
import { range, asapScheduler } from 'rxjs';
import { observeOn, bufferCount, tap } from 'rxjs';

// UI erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'observeOn - UI-Blockierungsvermeidung';
container.appendChild(title);

const progress = document.createElement('div');
progress.style.marginBottom = '10px';
container.appendChild(progress);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

function addLog(message: string) {
  const logItem = document.createElement('div');
  logItem.style.fontSize = '12px';
  logItem.style.marginBottom = '2px';
  logItem.textContent = message;
  output.appendChild(logItem);
}

const totalItems = 10000;
const batchSize = 100;
const totalBatches = Math.ceil(totalItems / batchSize);
let processedBatches = 0;

addLog('Verarbeitung gestartet...');
progress.textContent = 'Fortschritt: 0%';

range(1, totalItems)
  .pipe(
    bufferCount(batchSize),
    observeOn(asapScheduler),  // Jeden Batch asynchron verarbeiten
    tap(batch => {
      // Schwere Berechnung simulieren
      const sum = batch.reduce((acc, n) => acc + n, 0);
      processedBatches++;
      const percent = Math.floor((processedBatches / totalBatches) * 100);
      progress.textContent = `Fortschritt: ${percent}%`;

      if (processedBatches % 10 === 0 || processedBatches === totalBatches) {
        addLog(`Batch ${processedBatches}/${totalBatches} abgeschlossen (Summe: ${sum})`);
      }
    })
  )
  .subscribe({
    complete: () => {
      addLog('--- Alle Verarbeitungen abgeschlossen ---');
      progress.textContent = 'Fortschritt: 100% ✅';
    }
  });
```

- 10.000 Datensätze in Batches von 100 verarbeiten
- UI wird mit `asapScheduler` nicht blockiert
- Echtzeit-Fortschrittsanzeige

## 🧪 Praktisches Codebeispiel 2: Animation-Optimierung

Beispiel für flüssige Animation mit `animationFrameScheduler`.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { observeOn, take, map } from 'rxjs';

// UI erstellen
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'observeOn - Animation';
container2.appendChild(title2);

const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = '#4CAF50';
box.style.position = 'relative';
box.style.transition = 'none';
container2.appendChild(box);

let position = 0;

interval(0)
  .pipe(
    observeOn(animationFrameScheduler),  // Mit 60fps ausführen
    take(180),  // 3 Sekunden (60fps × 3 Sekunden)
    map(() => {
      position += 2;  // 2px pro Frame bewegen
      return position;
    })
  )
  .subscribe({
    next: pos => {
      box.style.left = `${pos}px`;
    },
    complete: () => {
      const message = document.createElement('div');
      message.textContent = 'Animation abgeschlossen';
      message.style.marginTop = '10px';
      message.style.color = '#4CAF50';
      container2.appendChild(message);
    }
  });
```

- Synchronisiert mit dem Render-Zyklus des Browsers durch `animationFrameScheduler`
- Flüssige 60fps-Animation
- Automatisches Pausieren in Hintergrund-Tabs

## 🆚 Unterschied zu subscribeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

console.log('=== observeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('Vor observeOn (synchron)')),
    observeOn(asyncScheduler),
    tap(() => console.log('Nach observeOn (asynchron)'))
  )
  .subscribe();

console.log('=== subscribeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('Nach subscribeOn (asynchron)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Ausgabe:
// === observeOn ===
// Vor observeOn (synchron)
// Vor observeOn (synchron)
// Vor observeOn (synchron)
// === subscribeOn ===
// Nach observeOn (asynchron)
// Nach observeOn (asynchron)
// Nach observeOn (asynchron)
// Nach subscribeOn (asynchron)
// Nach subscribeOn (asynchron)
// Nach subscribeOn (asynchron)
```

| Operator | Wirkungsbereich | Timing-Steuerung |
|:---|:---|:---|
| `observeOn` | Nur nachfolgende Verarbeitung | Emissionstiming von Werten |
| `subscribeOn` | Gesamter Stream | Subscription-Start-Timing |

> [!NOTE]
> Details zu `subscribeOn` finden Sie unter [subscribeOn](./subscribeOn.md).

## ⚠️ Wichtige Hinweise

### 1. Platzierung ist wichtig

Je nach Platzierung von `observeOn` ändert sich, welche Verarbeitung asynchron gemacht wird.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, map, tap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Verarbeitung 1 (synchron)')),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Ab hier asynchron
    tap(() => console.log('Verarbeitung 2 (asynchron)')),
    map(x => x + 10)
  )
  .subscribe();

// Verarbeitung 1 ist synchron, Verarbeitung 2 ist asynchron
```

### 2. Mehrfache observeOn werden nicht kumuliert

```ts
import { of, asyncScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    observeOn(queueScheduler)  // Der letzte Scheduler wird angewendet
  )
  .subscribe();
```

Der Scheduler des letzten `observeOn` (in diesem Fall `queueScheduler`) wird verwendet.

### 3. Auswirkung auf die Leistung

Häufige Verwendung von `observeOn` führt zu Overhead.

```ts
import { asyncScheduler, range, map, bufferCount, concatMap, from } from 'rxjs';
import { observeOn } from 'rxjs';

// ❌ Schlechtes Beispiel: Asynchron für jeden Wert
range(1, 1000)
  .pipe(
    map(x => x * 2),
    observeOn(asyncScheduler)  // 1000 setTimeouts
  )
  .subscribe();

// ✅ Gutes Beispiel: Batch-Verarbeitung
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeouts
    concatMap(batch => from(batch).pipe(map(x => x * 2)))
  )
  .subscribe();
```

## Vergleich des Ausführungstimings

```ts
import { of, asyncScheduler, asapScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

// Synchrone Verarbeitung
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(v => console.log(`5: ${v}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Ende');

// Ausführungsreihenfolge:
// 1: Start
// 2: sync
// 7: Ende
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```

## 📚 Verwandte Operatoren

- **[subscribeOn](./subscribeOn)** - Timing-Steuerung des Subscription-Starts
- **[delay](./delay)** - Feste Zeitverzögerung
- **[debounceTime](/de/guide/operators/filtering/debounceTime)** - Verzögerung nach Eingabestopp

## 📖 Verwandte Dokumentation

- **[Steuerung asynchroner Verarbeitung](../../schedulers/async-control.md)** - Scheduler-Grundlagen
- **[Scheduler-Typen und Verwendung](../../schedulers/types.md)** - Details zu jedem Scheduler

## ✅ Zusammenfassung

Der `observeOn`-Operator steuert das Emissionstiming und den Ausführungskontext von Werten.

- ✅ Nachfolgende Verarbeitung wird mit angegebenem Scheduler ausgeführt
- ✅ Effektiv zur Vermeidung von UI-Blockierung
- ✅ Verwendung zur Animation-Optimierung
- ✅ Priorisierung von Verarbeitungen möglich
- ⚠️ Platzierung ist wichtig
- ⚠️ Auf Leistungs-Overhead achten
- ⚠️ Bei mehrfacher Verwendung wird der letzte Scheduler angewendet
