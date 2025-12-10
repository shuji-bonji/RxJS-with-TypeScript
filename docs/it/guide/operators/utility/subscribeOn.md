---
description: L'operatore subscribeOn controlla quando iniziare la sottoscrizione a Observable con lo scheduler specificato e cambia il contesto di esecuzione dell'intero stream.
---

# subscribeOn - Controlla Quando Iniziare la Sottoscrizione

L'operatore `subscribeOn` controlla il **timing di inizio sottoscrizione Observable e il contesto di esecuzione con lo scheduler specificato**. Influenza il timing di esecuzione dell'intero stream.

## 🔰 Sintassi e Operazione Base

Asincronizza l'inizio di una sottoscrizione specificando uno scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

console.log('Inizio');

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler)
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

L'inizio della sottoscrizione stessa è asincronizzato, quindi la chiamata a `subscribe()` ritorna immediatamente.

[🌐 Documentazione Ufficiale RxJS - subscribeOn](https://rxjs.dev/api/index/function/subscribeOn)

## 💡 Esempi di Utilizzo Tipici

- **Asincronizza processi di inizializzazione pesanti**: Ritarda l'inizio del caricamento dati, ecc.
- **Previeni freeze della UI**: Avvia sottoscrizioni in modo asincrono per mantenere la reattività
- **Dare priorità all'elaborazione**: Controlla il timing di avvio di più stream
- **Controllo timing nei test**: Controlla usando TestScheduler

## 🧪 Esempio di Codice Pratico 1: Asincronizza Elaborazione Inizializzazione Pesante

Questo è un esempio di avvio asincrono della lettura dati e inizializzazione.

```ts
import { Observable, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// Creazione UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'subscribeOn - Elaborazione inizializzazione pesante';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const timestamp = now.toLocaleTimeString('it-IT', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${timestamp}] ${message}`;
  output.appendChild(logItem);
}

// Simula elaborazione inizializzazione pesante
const heavyInit$ = new Observable<string>(subscriber => {
  addLog('Caricamento dati avviato...', '#fff9c4');

  // Simula elaborazione pesante
  let sum = 0;
  for (let i = 0; i < 10000000; i++) {
    sum += i;
  }

  addLog('Caricamento dati completato', '#c8e6c9');
  subscriber.next(`Risultato: ${sum}`);
  subscriber.complete();
});

addLog('Inizio sottoscrizione (UI operabile)', '#e3f2fd');

heavyInit$
  .pipe(
    subscribeOn(asyncScheduler)  // Asincronizza inizio sottoscrizione
  )
  .subscribe({
    next: result => addLog(`Ricevuto: ${result}`, '#c8e6c9'),
    complete: () => addLog('Completato', '#e3f2fd')
  });

addLog('Dopo richiesta sottoscrizione (esecuzione continua immediatamente)', '#e3f2fd');
```

- L'inizio della sottoscrizione è asincrono, la UI risponde immediatamente
- L'elaborazione pesante viene eseguita in modo asincrono
- Il thread principale non viene bloccato

## 🧪 Esempio di Codice Pratico 2: Controllo Priorità Stream Multipli

Questo è un esempio di controllo del timing di avvio di più stream.

```ts
import { interval, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn, take, tap } from 'rxjs';

// Creazione UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'subscribeOn - Controllo priorità';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string, color: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('it-IT', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.marginBottom = '2px';
  logItem.style.backgroundColor = color;
  logItem.style.fontSize = '12px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Inizio', '#e3f2fd');

// Task ad alta priorità (asapScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asapScheduler),
    tap(v => addLog2(`Alta priorità: ${v}`, '#c8e6c9'))
  )
  .subscribe();

// Task a priorità normale (asyncScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asyncScheduler),
    tap(v => addLog2(`Priorità normale: ${v}`, '#fff9c4'))
  )
  .subscribe();

addLog2('Richiesta sottoscrizione completata', '#e3f2fd');
```

- Diversi scheduler controllano le priorità
- `asapScheduler` inizia l'esecuzione prima di `asyncScheduler`

## 🆚 Differenze da observeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

// Esempio observeOn
console.log('=== observeOn ===');
console.log('1: Inizio');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('2: tap (sync)')),
    observeOn(asyncScheduler),
    tap(() => console.log('4: tap (async)'))
  )
  .subscribe(() => console.log('5: subscribe'));

console.log('3: Fine');

// Esempio subscribeOn
console.log('\n=== subscribeOn ===');
console.log('1: Inizio');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('3: tap (async)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(() => console.log('4: subscribe'));

console.log('2: Fine');
```

**Differenze principali**:

| Elemento | observeOn | subscribeOn |
|:---|:---|:---|
| **Ambito degli Effetti** | Solo elaborazione successiva | Intero stream |
| **Target del Controllo** | Timing di pubblicazione valore | Timing di inizio sottoscrizione |
| **Posizionamento** | Importante (comportamento cambia in base a dove lo piazzi) | Uguale ovunque lo piazzi |
| **Uso Multiplo** | L'ultimo si applica | Il primo si applica |

> [!NOTE]
> Per maggiori informazioni su `observeOn`, vedi [observeOn](./observeOn.md).

## ⚠️ Note Importanti

### 1. La Posizione di Piazzamento Non Ha Effetto

`subscribeOn` ha lo stesso effetto indipendentemente da dove lo piazzi nella pipeline.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, map } from 'rxjs';

// Pattern 1: Primo
of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),
    map(x => x * 2)
  )
  .subscribe();

// Pattern 2: Ultimo
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Entrambi funzionano allo stesso modo
```

### 2. Più subscribeOn Applicano il Primo

```ts
import { of, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),  // Questo viene usato
    subscribeOn(asapScheduler)    // Questo viene ignorato
  )
  .subscribe();
```

Il primo scheduler `subscribeOn` (`asyncScheduler`) viene usato.

### 3. Alcuni Observable Non Hanno Effetto

Observable con il proprio scheduler, come `interval` e `timer`, non sono influenzati da `subscribeOn`.

```ts
import { interval, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// ❌ subscribeOn non ha effetto
interval(1000)
  .pipe(
    subscribeOn(asyncScheduler)  // interval usa il proprio scheduler
  )
  .subscribe();

// ✅ Specifica lo scheduler nell'argomento di interval
interval(1000, asyncScheduler)
  .subscribe();
```

## Esempi di Combinazione Pratica

```ts
import { of, asyncScheduler, animationFrameScheduler } from 'rxjs';
import { subscribeOn, observeOn, map, tap } from 'rxjs';

console.log('Inizio');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Tap 1 (async)')),
    subscribeOn(asyncScheduler),        // Asincronizza inizio sottoscrizione
    map(x => x * 2),
    observeOn(animationFrameScheduler), // Sincronizza emissione valore con animation frame
    tap(() => console.log('Tap 2 (animation frame)'))
  )
  .subscribe(v => console.log('Valore:', v));

console.log('Fine');

// Ordine di esecuzione:
// Inizio
// Fine
// Tap 1 (async)
// Tap 1 (async)
// Tap 1 (async)
// Tap 2 (animation frame)
// Valore: 2
// ... (continua sotto)
```

## Linee Guida per l'Utilizzo

### Caso 1: Vuoi Ritardare l'Inizio delle Sottoscrizioni
```ts
// → usa subscribeOn
of(data)
  .pipe(subscribeOn(asyncScheduler))
  .subscribe();
```

### Caso 2: Voglio Rendere Asincrono un Processo Specifico
```ts
// → usa observeOn
of(data)
  .pipe(
    map(elaborazione pesante),
    observeOn(asyncScheduler),  // Asincronizza solo dopo elaborazione pesante
    map(elaborazione leggera)
  )
  .subscribe();
```

### Caso 3: Asincronizza l'Intero Processo + Controlla Ulteriormente una Parte
```ts
// → usa subscribeOn + observeOn insieme
of(data)
  .pipe(
    subscribeOn(asyncScheduler),           // Asincronizza intero processo
    map(elaborazione 1),
    observeOn(animationFrameScheduler),    // Cambia per animazione
    map(elaborazione 2)
  )
  .subscribe();
```

## 📚 Operatori Correlati

- **[observeOn](./observeOn)** - Controlla quando i valori vengono emessi
- **[delay](./delay)** - Ritardo di tempo fisso

## 📖 Documenti Correlati

- **[Controllo Elaborazione Asincrona](/it/guide/schedulers/async-control.md)** - Basi degli Scheduler
- **[Tipi e Utilizzo degli Scheduler](/it/guide/schedulers/types.md)** - Dettagli di ogni scheduler

## ✅ Riepilogo

L'operatore `subscribeOn` controlla il timing e il contesto di esecuzione per l'inizio della sottoscrizione.

- ✅ Asincronizza l'inizio della sottoscrizione per l'intero stream
- ✅ Utile per asincronizzare processi di inizializzazione pesanti
- ✅ Utile per prevenire freeze della UI
- ✅ La posizione di piazzamento non ha effetto
- ⚠️ Quando si usano più Observable, il primo viene applicato
- ⚠️ Non efficace per alcuni Observable
- ⚠️ Scopo diverso da `observeOn`
