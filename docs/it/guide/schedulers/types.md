---
description: Spiega in dettaglio le caratteristiche, l'implementazione e l'utilizzo dei principali scheduler in RxJS, come asyncScheduler e queueScheduler. Comprendi le differenze tra macro task, micro task ed elaborazione sincrona e apprendi i tempi di esecuzione e le caratteristiche di ciascuno scheduler. La scelta appropriata ottimizza le prestazioni e il comportamento dell'applicazione.
---

# Tipi di Scheduler e Come Utilizzarli

RxJS fornisce diversi scheduler per scopi differenti. Ogni scheduler ha tempi di esecuzione e caratteristiche uniche e, utilizzandoli in modo appropriato, è possibile ottimizzare le prestazioni e il comportamento dell'applicazione.

## Classificazione degli Scheduler

Gli scheduler di RxJS sono classificati in tre grandi categorie:

1. **Macro Task**: Eseguiti nella successiva coda di task dell'event loop
2. **Micro Task**: Eseguiti immediatamente dopo il completamento del task corrente, prima dell'inizio del task successivo
3. **Elaborazione Sincrona**: Esecuzione immediata

Per maggiori dettagli, consultare anche [Concetti Base di Task e Scheduler](./task-and-scheduler-basics.md).

## Scheduler Principali

### asyncScheduler

#### Caratteristiche
- **Implementazione Interna**: Utilizza setTimeout
- **Timing di Esecuzione**: Macro task
- **Utilizzo**: Elaborazione asincrona generale, elaborazione che comporta il passare del tempo

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inizio');

of('Elaborazione asincrona')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fine');

// Output:
// 1: Inizio
// 2: Fine
// 3: Elaborazione asincrona
```

#### Casi d'Uso

```ts
import { asyncScheduler, map, observeOn, of } from "rxjs";

function heavyComputation(value: number): number {
  // Simula calcoli pesanti
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result = Math.sin(result);
  }
  return result;
}

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(value => heavyComputation(value))
  )
  .subscribe(result => {
    console.log(`Risultato calcolo: ${result}`);
  });
```

### queueScheduler

#### Caratteristiche
- **Implementazione Interna**: Coda di micro task
- **Timing di Esecuzione**: All'interno del task corrente (appare sincrono)
- **Utilizzo**: Accodamento di task, ottimizzazione di elaborazioni ricorsive

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inizio');

of('Elaborazione coda')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: Fine');

// Output:
// 1: Inizio
// 2: Elaborazione coda
// 3: Fine
```

#### Casi d'Uso

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs';

// Ottimizzazione di elaborazioni ricorsive
function fibonacci(n: number): Observable<number> {
  return of([0, 1]).pipe(
    observeOn(queueScheduler),
    expand(([a, b]) => of([b, a + b])),
    map(([a]) => a),
    take(n)
  );
}

fibonacci(10).subscribe(value => console.log(value));
```

### asapScheduler

#### Caratteristiche
- **Implementazione Interna**: Promise.resolve().then() o setImmediate
- **Timing di Esecuzione**: Micro task
- **Utilizzo**: Quando si desidera eseguire in modo asincrono il prima possibile

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inizio');

of('Elaborazione ASAP')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fine');

// Output:
// 1: Inizio
// 2: Fine
// 3: Elaborazione ASAP
```

#### Casi d'Uso

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Ottimizzazione degli eventi di movimento del mouse
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // Elaborazione aggiornamento UI
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Caratteristiche
- **Implementazione Interna**: requestAnimationFrame
- **Timing di Esecuzione**: Prima del prossimo rendering dello schermo
- **Utilizzo**: Animazioni, elaborazione rendering per 60fps

#### Esempio di Animazione di Rotazione Semplice

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// Crea elemento HTML
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Configurazione animazione
let rotation = 0;

// Animazione di 2 secondi a 60fps
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2 secondi = 120 frame
    map(() => {
      rotation += 3;  // Ruota di 3 gradi per frame
      return rotation;
    })
  )
  .subscribe(angle => {
    // Ruota effettivamente l'elemento DOM
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Perché è Necessario animationFrameScheduler

`animationFrameScheduler` esegue l'elaborazione sincronizzandola con il ciclo di rendering del browser, offrendo i seguenti vantaggi:

1. **Animazioni Fluide**: Poiché l'elaborazione viene eseguita in sincronia con il timing di rendering del browser (normalmente 60fps), si ottengono animazioni fluide senza scatti.
2. **Utilizzo Efficiente delle Risorse**: Quando il browser rende una scheda inattiva, l'esecuzione di requestAnimationFrame viene automaticamente sospesa, prevenendo l'uso inutile della CPU.
3. **Prevenzione dello Sfarfallio dello Schermo**: Poiché i calcoli vengono completati sicuramente prima del rendering dello schermo, si previene lo sfarfallio e la visualizzazione di frame incompleti.

Di seguito un confronto tra `setInterval` e `animationFrameScheduler`:

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ Animazione inefficiente usando setInterval
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // Circa 60fps

// Problemi:
// - Non sincronizzato con il timing di rendering del browser
// - Continua ad essere eseguito anche nelle schede in background
// - Non può garantire 60fps precisi

// ✅ Animazione efficiente usando animationFrameScheduler
interval(0, animationFrameScheduler)
  .pipe(
    map(() => {
      position += 1;
      return position;
    })
  )
  .subscribe(pos => {
    element.style.transform = `translateX(${pos}px)`;
  });

// Vantaggi
// - Sincronizzato con il timing di rendering del browser
// - Automaticamente sospeso nelle schede in background
// - Realizza 60fps stabili
```


#### Esempio di Animazione di Inseguimento del Mouse

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Crea cerchio che segue
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Rende trasparenti gli eventi del mouse
document.body.appendChild(circle);

// Posizione corrente e posizione obiettivo
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Monitora eventi di movimento del mouse
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove')
  .pipe(
    map(event => ({
      x: event.clientX,
      y: event.clientY
    }))
  );

// Loop di animazione
interval(0, animationFrameScheduler)
  .pipe(
    withLatestFrom(mouseMove$),
    map(([_, mousePos]) => mousePos)
  )
  .subscribe(({ x, y }) => {
    // Imposta la posizione del mouse come obiettivo
    targetX = x;
    targetY = y;

    // Muove gradualmente dalla posizione corrente alla posizione obiettivo (easing)
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;

    // Aggiorna elemento DOM
    circle.style.left = `${currentX - 15}px`;  // Aggiusta alla posizione centrale
    circle.style.top = `${currentY - 15}px`;
  });
```

## Guida alla Scelta degli Scheduler

### Confronto per Timing di Esecuzione

```ts
import { of, asyncScheduler, queueScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Inizio');

// Elaborazione sincrona
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler (micro task)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler (micro task)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler (macro task)
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

### Criteri di Selezione per Scopo

| Scheduler | Caratteristiche | Utilizzo Appropriato |
|--------------|------|----------|
| asyncScheduler | Usa setTimeout, completamente asincrono | Elaborazioni che richiedono tempo, esecuzione ritardata |
| queueScheduler | Sincrono ma ottimizza la ricorsione | Elaborazioni ricorsive, gestione coda task |
| asapScheduler | Esecuzione asincrona il prima possibile | Gestione eventi, elaborazioni che richiedono risposta veloce |
| animationFrameScheduler | Sincronizzato con rendering schermo | Animazioni, aggiornamenti UI, sviluppo giochi |

## Esempi di Uso Pratico

### Elaborazione di Grandi Quantità di Dati

```ts
import { from, queueScheduler } from 'rxjs';
import { mergeMap, observeOn, tap } from 'rxjs';

interface ApiRequest {
  endpoint: string;
  id: number;
}

const requests: ApiRequest[] = [
  { endpoint: '/users', id: 1 },
  { endpoint: '/posts', id: 1 },
  { endpoint: '/comments', id: 1 },
];

// Mette le richieste in coda ed elabora in ordine
from(requests)
  .pipe(
    observeOn(queueScheduler),
    tap((req) => console.log(`Aggiunto alla coda: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // Simulazione richiesta API reale
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`Risultato di ${req.endpoint}/${req.id}`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`Completato: ${result}`));
```

### Elaborazione Messaggi WebSocket

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// Nota: Questo è pseudo-codice che mostra il concetto
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // Tratta come stringa
});

socket$
  .pipe(
    // Elaborazione messaggi che richiedono risposta veloce
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('Messaggio ricevuto:', msg);
}
```

### Controllo del Retry degli Errori

Utilizzando gli scheduler con l'operatore `retry`, è possibile controllare finemente il timing dei retry.

#### Controllo Base del Retry

L'opzione `delay` dell'operatore `retry` utilizza internamente `asyncScheduler` per controllare l'intervallo tra i retry.

```ts
import { throwError, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

// Simulazione chiamata API
function fetchData(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.7) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Network error'));
    })
  );
}

fetchData(1)
  .pipe(
    retry({
      count: 3,
      delay: 1000  // Attende 1 secondo con asyncScheduler prima del retry
    })
  )
  .subscribe({
    next: result => console.log('✅ Successo:', result),
    error: error => console.log('❌ Errore finale:', error.message)
  });
```

#### Utilizzo dello Scheduler con Exponential Backoff

Come controllo più avanzato, è possibile implementare l'exponential backoff combinando `retryWhen` e `asyncScheduler`.

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, mergeMap, tap } from 'rxjs';

function fetchDataWithBackoff(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.9) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Temporary error'));
    })
  );
}

fetchDataWithBackoff(1)
  .pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;

          // Controllo numero massimo retry
          if (retryCount > 3) {
            console.log('❌ Raggiunto numero massimo di retry');
            throw error;
          }

          // Exponential backoff: 1 secondo, 2 secondi, 4 secondi...
          const delayTime = Math.pow(2, index) * 1000;
          console.log(`🔄 Retry ${retryCount}° (dopo ${delayTime}ms)`);

          // timer utilizza internamente asyncScheduler
          return timer(delayTime);
        })
      )
    )
  )
  .subscribe({
    next: result => console.log('✅ Successo:', result),
    error: error => console.log('❌ Errore finale:', error.message)
  });

// Esempio di output:
// 🔄 Retry 1° (dopo 1000ms)
// 🔄 Retry 2° (dopo 2000ms)
// 🔄 Retry 3° (dopo 4000ms)
// ❌ Raggiunto numero massimo di retry
// ❌ Errore finale: Temporary error
```

#### Quando Specificare Esplicitamente asyncScheduler

Specificando esplicitamente uno scheduler specifico, è possibile un controllo più flessibile, come la sostituzione con `TestScheduler` durante i test.

```ts
import { throwError, asyncScheduler, of } from 'rxjs';
import { retryWhen, mergeMap, delay } from 'rxjs';

function fetchDataWithScheduler(id: number, scheduler = asyncScheduler) {
  return of(id).pipe(
    mergeMap(() => throwError(() => new Error('Error'))),
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          if (index >= 2) throw error;

          // Specifica esplicitamente lo scheduler
          return of(null).pipe(
            delay(1000, scheduler)
          );
        })
      )
    )
  );
}

// Ambiente di produzione: usa asyncScheduler
fetchDataWithScheduler(1).subscribe({
  error: err => console.log('Errore:', err.message)
});

// Ambiente di test: sostituibile con TestScheduler
```

> [!TIP]
> Per pattern di implementazione dettagliati e metodi di debug dell'elaborazione retry, consulta la pagina [retry e catchError](/it/guide/error-handling/retry-catch).
> - Uso dettagliato dell'operatore retry
> - Pattern di combinazione con catchError
> - Tecniche di debug del retry (tracciamento numero di tentativi, registrazione log, ecc.)

## Impatto sulle Prestazioni

### Overhead degli Scheduler

```ts
import { range, asyncScheduler, pipe } from 'rxjs';
import { bufferCount, map, observeOn, tap } from 'rxjs';

// ❌ Uso eccessivo dello scheduler
range(1, 1000)
  .pipe(
    observeOn(asyncScheduler),  // 1000 setTimeout
    map(x => x * 2),
    // tap(console.log)
  )
  .subscribe();

// ✅ Ottimizzazione con elaborazione batch
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeout
    map(batch => batch.map(x => x * 2)),
    // tap(console.log)
  )
  .subscribe();
```

## Riepilogo

La scelta dello scheduler ha un grande impatto sulle prestazioni e sulla reattività dell'applicazione. Comprendendo le caratteristiche di ogni scheduler e usandoli appropriatamente nelle situazioni giuste, è possibile realizzare un funzionamento efficiente e fluido. Come linee guida generali, si raccomanda di utilizzare:

- `asyncScheduler` per elaborazioni asincrone generali
- `queueScheduler` per elaborazioni ricorsive o accodamento sincrono
- `asapScheduler` quando è necessaria una risposta veloce
- `animationFrameScheduler` per animazioni
