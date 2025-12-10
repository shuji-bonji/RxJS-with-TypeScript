---
description: L'operatore observeOn controlla il timing di emissione dei valori Observable con uno scheduler specificato ed è usato per elaborazione asincrona e ottimizzazione animazioni.
---

# observeOn - Controllo del Contesto di Esecuzione

L'operatore `observeOn` controlla **il timing di emissione dei valori Observable e il contesto di esecuzione** con uno scheduler specificato. Le operazioni successive in uno stream possono essere fatte eseguire su uno scheduler specifico.

## 🔰 Sintassi e Operazione Base

Asincronizza l'elaborazione successiva specificando uno scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Inizio');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(v => console.log('Valore:', v));

console.log('Fine');

// Output:
// Inizio
// Fine
// Valore: 1
// Valore: 2
// Valore: 3
```

I processi prima di `observeOn` vengono eseguiti sincronamente, mentre i processi dopo `observeOn` vengono eseguiti dallo scheduler specificato.

[🌐 Documentazione Ufficiale RxJS - observeOn](https://rxjs.dev/api/index/function/observeOn)

## 💡 Esempi di Utilizzo Tipici

- **Evitare blocco del thread UI**: Asincronizza elaborazioni pesanti
- **Ottimizzazione animazioni**: Rendering fluido con `animationFrameScheduler`
- **Dare priorità all'elaborazione**: Controlla il timing di esecuzione con diversi scheduler
- **Controllo micro/macro task**: Regola fine del timing di esecuzione

## Tipi di Scheduler

| Scheduler | Caratteristiche | Casi d'Uso |
|:---|:---|:---|
| `asyncScheduler` | Basato su `setTimeout` | Elaborazione asincrona generale |
| `asapScheduler` | Microtask (Promise.then) | Esecuzione asincrona il più veloce possibile |
| `queueScheduler` | Coda sincrona | Ottimizza elaborazione ricorsiva |
| `animationFrameScheduler` | `requestAnimationFrame` | Animazione, rendering 60fps |

> [!TIP]
> Per maggiori informazioni sugli scheduler, vedi [Tipi di Scheduler e Come Usarli](/it/guide/schedulers/types.md).

## 🧪 Esempio di Codice Pratico 1: Evitare Blocco UI

Questo è un esempio di esecuzione asincrona di una grande quantità di elaborazione dati divisa in batch.

```ts
import { range, asapScheduler } from 'rxjs';
import { observeOn, bufferCount, tap } from 'rxjs';

// Creazione UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'observeOn - Evitare blocco UI';
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

addLog('Elaborazione avviata...');
progress.textContent = 'Progresso: 0%';

range(1, totalItems)
  .pipe(
    bufferCount(batchSize),
    observeOn(asapScheduler),  // Elabora ogni batch in modo asincrono
    tap(batch => {
      // Simula calcolo pesante
      const sum = batch.reduce((acc, n) => acc + n, 0);
      processedBatches++;
      const percent = Math.floor((processedBatches / totalBatches) * 100);
      progress.textContent = `Progresso: ${percent}%`;

      if (processedBatches % 10 === 0 || processedBatches === totalBatches) {
        addLog(`Batch ${processedBatches}/${totalBatches} completato (Totale: ${sum})`);
      }
    })
  )
  .subscribe({
    complete: () => {
      addLog('--- Tutta l\'elaborazione completata ---');
      progress.textContent = 'Progresso: 100% ✅';
    }
  });
```

- Elaborazione batch di 10.000 elementi di dati, 100 alla volta
- Elabora senza bloccare la UI con `asapScheduler`
- Visualizzazione in tempo reale del progresso

## 🧪 Esempio di Codice Pratico 2: Ottimizzazione Animazione

Esempio di animazione fluida usando `animationFrameScheduler`.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { observeOn, take, map } from 'rxjs';

// Creazione UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'observeOn - Animazione';
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
    observeOn(animationFrameScheduler),  // Esegui a 60fps
    take(180),  // 3 secondi (60fps × 3 secondi)
    map(() => {
      position += 2;  // Muovi 2px per frame
      return position;
    })
  )
  .subscribe({
    next: pos => {
      box.style.left = `${pos}px`;
    },
    complete: () => {
      const message = document.createElement('div');
      message.textContent = 'Animazione completata';
      message.style.marginTop = '10px';
      message.style.color = '#4CAF50';
      container2.appendChild(message);
    }
  });
```

- Sincronizza con i cicli di disegno del browser con `animationFrameScheduler`
- Animazione fluida a 60fps
- Pausa automatica nelle tab in background

## 🆚 Differenze da subscribeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

console.log('=== observeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('Prima di observeOn (sync)')),
    observeOn(asyncScheduler),
    tap(() => console.log('Dopo observeOn (async)'))
  )
  .subscribe();

console.log('=== subscribeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('Dopo subscribeOn (async)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Output:
// === observeOn ===
// Prima di observeOn (sync)
// Prima di observeOn (sync)
// Prima di observeOn (sync)
// === subscribeOn ===
// Dopo observeOn (async)
// Dopo observeOn (async)
// Dopo observeOn (async)
// Dopo subscribeOn (async)
// Dopo subscribeOn (async)
// Dopo subscribeOn (async)
```

| Operatore | Ambito degli Effetti | Controllo Timing |
|:---|:---|:---|
| `observeOn` | Solo processi successivi | Timing per emissione valore |
| `subscribeOn` | Intero stream | Timing per inizio subscription |

> [!NOTE]
> Per maggiori informazioni su `subscribeOn`, vedi [subscribeOn](./subscribeOn.md).

## ⚠️ Note Importanti

### 1. La Posizione di Piazzamento è Importante

La posizione di `observeOn` determina quali processi vengono asincronizzati.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, map, tap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Processo 1 (sync)')),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Async da qui
    tap(() => console.log('Processo 2 (async)')),
    map(x => x + 10)
  )
  .subscribe();

// Processo 1 è sincrono, Processo 2 è asincrono
```

### 2. Più observeOn Non Sono Cumulativi

```ts
import { of, asyncScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    observeOn(queueScheduler)  // L'ultimo scheduler viene applicato
  )
  .subscribe();
```

L'ultimo scheduler `observeOn` (in questo caso `queueScheduler`) viene usato.

### 3. Impatto sulle Performance

L'uso frequente di `observeOn` ha un overhead.

```ts
import { asyncScheduler, range, map, bufferCount, concatMap, from } from 'rxjs';
import { observeOn } from 'rxjs';

// ❌ Esempio sbagliato: Asincronizza per ogni valore
range(1, 1000)
  .pipe(
    map(x => x * 2),
    observeOn(asyncScheduler)  // 1000 setTimeout
  )
  .subscribe();

// ✅ Esempio corretto: Elaborazione batch
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeout
    concatMap(batch => from(batch).pipe(map(x => x * 2)))
  )
  .subscribe();
```

## Confronto Timing di Esecuzione

```ts
import { of, asyncScheduler, asapScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inizio');

// Elaborazione sincrona
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

console.log('7: Fine');

// Ordine di esecuzione:
// 1: Inizio
// 2: sync
// 7: Fine
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```

## 📚 Operatori Correlati

- **[subscribeOn](./subscribeOn)** - Controlla timing di inizio subscription
- **[delay](./delay)** - Ritardo di tempo fisso
- **[debounceTime](../filtering/debounceTime)** - Ritardo dopo che l'input si ferma

## 📖 Documenti Correlati

- **[Controllo Elaborazione Asincrona](/it/guide/schedulers/async-control.md)** - Basi degli Scheduler
- **[Tipi e Utilizzo degli Scheduler](/it/guide/schedulers/types.md)** - Dettagli di ogni scheduler

## ✅ Riepilogo

L'operatore `observeOn` controlla quando i valori vengono emessi e il contesto di esecuzione.

- ✅ Esegui processi successivi con lo scheduler specificato
- ✅ Utile per evitare blocchi UI
- ✅ Utilizzato per ottimizzazione animazioni
- ✅ Permette di dare priorità all'elaborazione
- ⚠️ La posizione di piazzamento è importante
- ⚠️ Fai attenzione all'overhead delle performance
- ⚠️ Quando usi più scheduler, l'ultimo scheduler viene applicato
