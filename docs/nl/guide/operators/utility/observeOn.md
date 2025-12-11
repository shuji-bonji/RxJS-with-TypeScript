---
description: De observeOn operator regelt de timing van het uitgeven van Observable-waarden met een gespecificeerde scheduler en wordt gebruikt voor asynchrone verwerking en animatie-optimalisatie.
---

# observeOn - Controle van uitvoeringscontext

De `observeOn` operator regelt **de timing van het uitgeven van Observable-waarden en de uitvoeringscontext** met een gespecificeerde scheduler. Volgende bewerkingen in een stream kunnen worden uitgevoerd op een specifieke scheduler.

## 🔰 Basissyntax en werking

Asynchroon maken van volgende verwerking door een scheduler te specificeren.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Start');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(v => console.log('Waarde:', v));

console.log('Einde');

// Uitvoer:
// Start
// Einde
// Waarde: 1
// Waarde: 2
// Waarde: 3
```

Processen vóór `observeOn` worden synchroon uitgevoerd, terwijl processen na `observeOn` worden uitgevoerd door de gespecificeerde scheduler.

[🌐 RxJS Officiële Documentatie - observeOn](https://rxjs.dev/api/index/function/observeOn)

## 💡 Typische gebruiksvoorbeelden

- **UI-thread blokkering vermijden**: Zware verwerking asynchroon maken
- **Optimalisatie van animatie**: Soepele rendering met `animationFrameScheduler`
- **Verwerking prioriteren**: Uitvoeringstiming regelen met verschillende schedulers
- **Micro/macro taak controle**: Uitvoeringstiming fijn afstemmen

## Soorten schedulers

| Scheduler | Kenmerken | Gebruikssituaties |
|:---|:---|:---|
| `asyncScheduler` | Gebaseerd op `setTimeout` | Algemene asynchrone verwerking |
| `asapScheduler` | Microtaken (Promise.then) | Zo snel mogelijke asynchrone uitvoering |
| `queueScheduler` | Synchrone wachtrij | Recursieve verwerking optimaliseren |
| `animationFrameScheduler` | `requestAnimationFrame` | Animatie, 60fps rendering |

> [!TIP]
> Voor meer informatie over schedulers, zie [Soorten schedulers en hoe ze te gebruiken](/nl/guide/schedulers/types.md).

## 🧪 Praktisch codevoorbeeld 1: UI-blokkering vermijden

Dit is een voorbeeld van asynchrone uitvoering van grote hoeveelheden dataverwerking verdeeld in batches.

```ts
import { range, asapScheduler } from 'rxjs';
import { observeOn, bufferCount, tap } from 'rxjs';

// UI creatie
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'observeOn - UI-blokkering vermijden';
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

addLog('Verwerking gestart...');
progress.textContent = 'Voortgang: 0%';

range(1, totalItems)
  .pipe(
    bufferCount(batchSize),
    observeOn(asapScheduler),  // Verwerk elke batch asynchroon
    tap(batch => {
      // Simuleer zware berekening
      const sum = batch.reduce((acc, n) => acc + n, 0);
      processedBatches++;
      const percent = Math.floor((processedBatches / totalBatches) * 100);
      progress.textContent = `Voortgang: ${percent}%`;

      if (processedBatches % 10 === 0 || processedBatches === totalBatches) {
        addLog(`Batch ${processedBatches}/${totalBatches} voltooid (Totaal: ${sum})`);
      }
    })
  )
  .subscribe({
    complete: () => {
      addLog('--- Alle verwerking voltooid ---');
      progress.textContent = 'Voortgang: 100% ✅';
    }
  });
```

- Batchverwerking van 10.000 data-items, 100 tegelijk
- Verwerk zonder UI te blokkeren met `asapScheduler`
- Realtime weergave van voortgang

## 🧪 Praktisch codevoorbeeld 2: Animatie-optimalisatie

Voorbeeld van soepele animatie met `animationFrameScheduler`.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { observeOn, take, map } from 'rxjs';

// UI creatie
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'observeOn - Animatie';
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
    observeOn(animationFrameScheduler),  // Uitvoeren op 60fps
    take(180),  // 3 seconden (60fps × 3 seconden)
    map(() => {
      position += 2;  // Verplaats 2px per frame
      return position;
    })
  )
  .subscribe({
    next: pos => {
      box.style.left = `${pos}px`;
    },
    complete: () => {
      const message = document.createElement('div');
      message.textContent = 'Animatie voltooid';
      message.style.marginTop = '10px';
      message.style.color = '#4CAF50';
      container2.appendChild(message);
    }
  });
```

- Synchroniseer met browser-tekencycli met `animationFrameScheduler`
- Soepele 60fps animatie
- Automatisch pauzeren op achtergrondtabbladen

## 🆚 Verschillen met subscribeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

console.log('=== observeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('Vóór observeOn (sync)')),
    observeOn(asyncScheduler),
    tap(() => console.log('Na observeOn (async)'))
  )
  .subscribe();

console.log('=== subscribeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('Na subscribeOn (async)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Uitvoer:
// === observeOn ===
// Vóór observeOn (sync)
// Vóór observeOn (sync)
// Vóór observeOn (sync)
// === subscribeOn ===
// Na observeOn (async)
// Na observeOn (async)
// Na observeOn (async)
// Na subscribeOn (async)
// Na subscribeOn (async)
// Na subscribeOn (async)
```

| Operator | Bereik van effecten | Timing controle |
|:---|:---|:---|
| `observeOn` | Alleen volgende processen | Timing voor het uitgeven van waarde |
| `subscribeOn` | Gehele stream | Timing voor het starten van abonnement |

> [!NOTE]
> Voor meer informatie over `subscribeOn`, zie [subscribeOn](./subscribeOn.md).

## ⚠️ Belangrijke opmerkingen

### 1. Plaatsingspositie is belangrijk

De locatie van `observeOn` bepaalt welke processen asynchroon worden gemaakt.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, map, tap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Proces 1 (sync)')),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Async vanaf hier
    tap(() => console.log('Proces 2 (async)')),
    map(x => x + 10)
  )
  .subscribe();

// Proces 1 is synchroon, Proces 2 is asynchroon
```

### 2. Meerdere observeOn zijn niet cumulatief

```ts
import { of, asyncScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    observeOn(queueScheduler)  // Laatste scheduler wordt toegepast
  )
  .subscribe();
```

De laatste `observeOn` scheduler (in dit geval `queueScheduler`) wordt gebruikt.

### 3. Prestatie-impact

Frequent gebruik van `observeOn` heeft overhead.

```ts
import { asyncScheduler, range, map, bufferCount, concatMap, from } from 'rxjs';
import { observeOn } from 'rxjs';

// ❌ Slecht voorbeeld: Asynchroon maken voor elke waarde
range(1, 1000)
  .pipe(
    map(x => x * 2),
    observeOn(asyncScheduler)  // 1000 setTimeouts
  )
  .subscribe();

// ✅ Goed voorbeeld: Batchverwerking
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeouts
    concatMap(batch => from(batch).pipe(map(x => x * 2)))
  )
  .subscribe();
```

## Vergelijking van uitvoeringstiming

```ts
import { of, asyncScheduler, asapScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Start');

// Synchrone verwerking
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

console.log('7: Einde');

// Uitvoeringsvolgorde:
// 1: Start
// 2: sync
// 7: Einde
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```

## 📚 Gerelateerde operators

- **[subscribeOn](./subscribeOn)** - Regel timing van abonnementstart
- **[delay](./delay)** - Vaste tijdvertraging
- **[debounceTime](../filtering/debounceTime)** - Vertraging na invoer stopt

## 📖 Gerelateerde documenten

- **[Controle van asynchrone verwerking](/nl/guide/schedulers/async-control.md)** - Scheduler basis
- **[Soorten en gebruik van schedulers](/nl/guide/schedulers/types.md)** - Details van elke scheduler

## ✅ Samenvatting

De `observeOn` operator regelt wanneer waarden worden uitgegeven en de uitvoeringscontext.

- ✅ Voer volgende processen uit met de gespecificeerde scheduler
- ✅ Nuttig voor het vermijden van UI-blokkering
- ✅ Gebruikt voor animatie-optimalisatie
- ✅ Maakt prioritering van verwerking mogelijk
- ⚠️ Plaatsingspositie is belangrijk
- ⚠️ Wees bewust van prestatie-overhead
- ⚠️ Bij gebruik van meerdere schedulers wordt de laatste scheduler toegepast
