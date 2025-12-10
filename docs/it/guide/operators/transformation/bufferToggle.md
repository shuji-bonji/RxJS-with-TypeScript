---
description: L'operatore bufferToggle è un operatore di buffering avanzato che permette di controllare i trigger di inizio e fine con Observable separati e gestire più periodi di buffering indipendentemente.
titleTemplate: ':title'
---

# bufferToggle - Buffer con Controllo Indipendente di Inizio e Fine

L'operatore `bufferToggle` controlla il **trigger di inizio** e il **trigger di fine** con Observable separati ed emette i valori in un array. Questo è un operatore di buffering avanzato che può gestire più periodi di buffering simultaneamente.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // Emetti valori ogni 0.5 secondi

// Trigger di inizio: ogni 2 secondi
const opening$ = interval(2000);

// Trigger di fine: 1 secondo dopo l'inizio
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output:
// [3, 4, 5]     (Inizia a 2 sec, finisce a 3 sec)
// [7, 8, 9]     (Inizia a 4 sec, finisce a 5 sec)
// [11, 12, 13]  (Inizia a 6 sec, finisce a 7 sec)
```

**Flusso di operazione**:
1. `opening$` emette un valore → Il buffering inizia
2. L'Observable restituito da `closing()` emette un valore → Il buffering finisce, emette array
3. Più periodi di buffering possono sovrapporsi

[🌐 Documentazione Ufficiale RxJS - `bufferToggle`](https://rxjs.dev/api/operators/bufferToggle)

## 🆚 Confronto con Altri Operatori Buffer

`bufferToggle` è unico rispetto ad altri operatori buffer in quanto permette il **controllo indipendente** di inizio e fine.

### Confronto di Ogni Operatore

| Operatore | Trigger | Caratteristica | Caso d'Uso |
|---|---|---|---|
| `buffer(trigger$)` | Singolo Observable | Semplice | Buffering guidato da eventi |
| `bufferTime(ms)` | Tempo | Periodico | Aggregazione dati a intervalli regolari |
| `bufferCount(n)` | Conteggio | Quantitativo | Elaborazione in unità di N |
| `bufferToggle(open$, close)` | Controllo separato inizio/fine | Flessibile | Gestione periodi complessi |

### Confronto Esempio di Codice

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // Emetti 0-9 ogni 300ms

// bufferToggle: Controllo indipendente di inizio e fine
const opening$ = interval(1000); // Inizio ogni 1 secondo
const closing = () => interval(500); // Fine 500ms dopo l'inizio

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output: [3, 4], [6, 7], [9]
//
// Timeline:
// 0ms  300ms 600ms 900ms 1200ms 1500ms 1800ms 2100ms 2400ms 2700ms
// 0    1     2     3     4      5      6      7      8      9
//                  [Inizio      Fine]   [Inizio      Fine]   [Inizio Fine]
//                  └→ [3,4]            └→ [6,7]            └→ [9]
```

**Linee guida di utilizzo**:
- **`buffer`** → Emetti buffer ogni volta che l'Observable trigger emette un valore
- **`bufferTime`** → Emetti automaticamente buffer a intervalli regolari
- **`bufferCount`** → Emetti buffer quando viene raggiunto il conteggio specificato
- **`bufferToggle`** → Controllo separato inizio/fine, periodi sovrapposti possibili

> [!TIP]
> Per maggiori dettagli su ogni operatore, vedi [buffer](/it/guide/operators/transformation/buffer), [bufferTime](/it/guide/operators/transformation/bufferTime), [bufferCount](/it/guide/operators/transformation/bufferCount).

## 💡 Pattern di Utilizzo Tipici

1. **Raccolta Dati Durante Orario Lavorativo**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // Dati sensore (acquisizione sempre attiva)
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       value: Math.random() * 100
     }))
   );

   // Apertura attività: 9:00 (Simulazione: dopo 2 secondi)
   const businessOpen$ = timer(2000, 10000); // Dopo 2 sec, poi ogni 10 sec

   // Chiusura attività: 5 secondi dopo l'inizio
   const businessClose = () => timer(5000);

   sensorData$.pipe(
     bufferToggle(businessOpen$, businessClose)
   ).subscribe(data => {
     console.log(`Dati durante orario lavorativo: ${data.length} elementi`);
     console.log(`Media: ${(data.reduce((sum, d) => sum + d.value, 0) / data.length).toFixed(2)}`);
   });
   ```

2. **Registrazione Eventi Durante Pressione Bottone**
   ```ts
   import { fromEvent, interval } from 'rxjs';
   import { bufferToggle, map, take } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Tieni premuto';
   document.body.appendChild(button);

   const display = document.createElement('div');
   display.style.marginTop = '10px';
   document.body.appendChild(display);

   // Stream dati
   const data$ = interval(100).pipe(
     map(i => ({ id: i, timestamp: Date.now() }))
   );

   // Inizio: Mouse giù
   const mouseDown$ = fromEvent(button, 'mousedown');

   // Fine: Mouse su (da mousedown a mouseup)
   const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

   data$.pipe(
     bufferToggle(mouseDown$, mouseUp)
   ).subscribe(events => {
     display.textContent = `Eventi registrati durante la pressione: ${events.length} elementi`;
     console.log('Eventi registrati:', events);
   });
   ```

3. **Registrazione Azioni Utente Attivo**
   ```ts
   import { fromEvent, merge, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // Azioni utente
   const clicks$ = fromEvent(document, 'click').pipe(
     map(() => ({ type: 'click' as const, timestamp: Date.now() }))
   );

   const scrolls$ = fromEvent(window, 'scroll').pipe(
     map(() => ({ type: 'scroll' as const, timestamp: Date.now() }))
   );

   const keypresses$ = fromEvent(document, 'keypress').pipe(
     map(() => ({ type: 'keypress' as const, timestamp: Date.now() }))
   );

   const actions$ = merge(clicks$, scrolls$, keypresses$);

   // Inizio stato attivo: prima azione
   const activeStart$ = actions$;

   // Fine stato attivo: nessuna azione per 5 secondi
   const activeEnd = () => timer(5000);

   actions$.pipe(
     bufferToggle(activeStart$, activeEnd)
   ).subscribe(bufferedActions => {
     console.log(`Sessione attiva: ${bufferedActions.length} azioni`);
     const summary = bufferedActions.reduce((acc, action) => {
       acc[action.type] = (acc[action.type] || 0) + 1;
       return acc;
     }, {} as Record<string, number>);
     console.log('Dettaglio:', summary);
   });
   ```

## 🧠 Esempio di Codice Pratico (Gestione Periodo Download)

Questo è un esempio di gestione dei periodi di download dati con bottoni di avvio e stop.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { bufferToggle, map, take } from 'rxjs';

// Crea elementi UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Gestione Download Dati';
container.appendChild(title);

const startButton = document.createElement('button');
startButton.textContent = 'Avvia';
container.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Stop';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
container.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'In attesa...';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// Stream dati (genera dati di download ogni 1 secondo)
const downloadData$ = interval(1000).pipe(
  map(i => ({
    id: i,
    size: Math.floor(Math.random() * 1000) + 100, // 100-1100KB
    timestamp: new Date()
  }))
);

// Trigger di avvio e stop
const start$ = fromEvent(startButton, 'click');
const stop$ = new Subject<void>();

fromEvent(stopButton, 'click').subscribe(() => {
  stop$.next();
  status.textContent = 'Fermato';
  startButton.disabled = false;
  stopButton.disabled = true;
});

start$.subscribe(() => {
  status.textContent = 'Download in corso...';
  startButton.disabled = true;
  stopButton.disabled = false;
});

// Buffering
downloadData$.pipe(
  bufferToggle(start$, () => stop$)
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Download Completato</strong><br>
    Conteggio: ${downloads.length} elementi<br>
    Dimensione totale: ${(totalSize / 1024).toFixed(2)} MB<br>
    Dimensione media: ${avgSize.toFixed(0)} KB
  `;

  console.log('Dati download:', downloads);
});
```

## 🎯 Periodi di Buffer Sovrapposti

Una caratteristica di `bufferToggle` è che può gestire più periodi di buffering simultaneamente.

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Inizio: ogni 1 secondo
const opening$ = interval(1000);

// Fine: 1.5 secondi dopo l'inizio
const closing = () => interval(1500);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output:
// [4, 5, 6]        (Inizia a 1 sec → Finisce a 2.5 sec)
// [9, 10, 11, 12]  (Inizia a 2 sec → Finisce a 3.5 sec) ※Parzialmente sovrapposto
// [14, 15, 16, 17] (Inizia a 3 sec → Finisce a 4.5 sec)
```

**Timeline**:
```
Sorgente:  0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Inizio:    ----1sec----2sec----3sec----4sec
Periodo 1: [------1.5sec-----]
            └→ Output: [4,5,6]
Periodo 2:        [------1.5sec-----]
                   └→ Output: [9,10,11,12]
Periodo 3:               [------1.5sec-----]
                          └→ Output: [14,15,16,17]
```

## 📋 Utilizzo Type-Safe

Ecco un esempio di implementazione type-safe utilizzando i generics in TypeScript.

```ts
import { Observable, Subject, interval } from 'rxjs';
import { bufferToggle, map } from 'rxjs';

interface MetricData {
  timestamp: Date;
  cpu: number;
  memory: number;
}

interface SessionControl {
  start$: Observable<void>;
  stop$: Observable<void>;
}

class MetricsCollector {
  private startSubject = new Subject<void>();
  private stopSubject = new Subject<void>();

  start(): void {
    this.startSubject.next();
  }

  stop(): void {
    this.stopSubject.next();
  }

  collectMetrics(source$: Observable<MetricData>): Observable<MetricData[]> {
    return source$.pipe(
      bufferToggle(
        this.startSubject,
        () => this.stopSubject
      )
    );
  }
}

// Esempio di utilizzo
const metricsStream$ = interval(500).pipe(
  map(() => ({
    timestamp: new Date(),
    cpu: Math.random() * 100,
    memory: Math.random() * 100
  } as MetricData))
);

const collector = new MetricsCollector();

collector.collectMetrics(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avgCpu = metrics.reduce((sum, m) => sum + m.cpu, 0) / metrics.length;
    const avgMemory = metrics.reduce((sum, m) => sum + m.memory, 0) / metrics.length;
    console.log(`Periodo di raccolta: ${metrics.length} elementi`);
    console.log(`CPU media: ${avgCpu.toFixed(1)}%`);
    console.log(`Memoria media: ${avgMemory.toFixed(1)}%`);
  }
});

// Avvia dopo 3 secondi
setTimeout(() => {
  console.log('Avvio raccolta');
  collector.start();
}, 3000);

// Ferma dopo 6 secondi
setTimeout(() => {
  console.log('Stop raccolta');
  collector.stop();
}, 6000);
```

## 🔄 Differenza da bufferWhen

`bufferToggle` e `bufferWhen` sono simili, ma i metodi di controllo sono diversi.

```ts
import { interval, timer } from 'rxjs';
import { bufferToggle, bufferWhen } from 'rxjs';

const source$ = interval(200);

// bufferToggle: Controllo separato di inizio e fine
source$.pipe(
  bufferToggle(
    interval(1000),          // Trigger di inizio
    () => timer(500)         // Trigger di fine (500ms dopo l'inizio)
  )
).subscribe(console.log);

// bufferWhen: Controlla solo il timing di fine (il prossimo inizia subito dopo la fine)
source$.pipe(
  bufferWhen(() => timer(1000)) // Buffer ogni 1 secondo
).subscribe(console.log);
```

| Operatore | Controllo | Periodo Buffer | Caso d'Uso |
|---|---|---|---|
| `bufferToggle(open$, close)` | Controllo separato inizio/fine | Può sovrapporsi | Condizioni complesse di inizio/fine |
| `bufferWhen(closing)` | Solo controllo fine | Continuo | Buffer periodico semplice |

## ⚠️ Errori Comuni

> [!WARNING]
> `bufferToggle` può gestire più periodi di buffer simultaneamente, ma se i trigger di inizio si attivano troppo frequentemente, molti buffer esisteranno allo stesso tempo, consumando memoria.

### Errore: Trigger di Inizio Troppo Frequenti

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ❌ Esempio sbagliato: Inizio ogni 100ms, fine dopo 5 secondi
const opening$ = interval(100); // Troppo frequente
const closing = () => interval(5000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Fino a 50 buffer possono esistere simultaneamente → Rischio memoria
```

### Corretto: Imposta un Intervallo Appropriato

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ✅ Esempio corretto: Imposta intervallo appropriato per l'inizio
const opening$ = interval(2000); // Ogni 2 secondi
const closing = () => interval(1000); // Buffer per 1 secondo

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Al massimo 1-2 buffer esistono simultaneamente
```

## 🎓 Riepilogo

### Quando Usare bufferToggle
- ✅ Quando vuoi controllare inizio e fine indipendentemente
- ✅ Quando vuoi raccogliere dati per un periodo limitato, come durante la pressione di un bottone
- ✅ Quando vuoi gestire più periodi di buffering simultaneamente
- ✅ Raccolta dati in condizioni complesse, come solo durante l'orario lavorativo

### Quando Usare buffer/bufferTime/bufferCount
- ✅ Quando è sufficiente un buffering periodico semplice
- ✅ Quando un singolo trigger è sufficiente per il controllo

### Quando Usare bufferWhen
- ✅ Quando solo la condizione di fine deve essere controllata dinamicamente
- ✅ Quando sono necessari periodi di buffering continui

### Note
- ⚠️ Trigger di inizio frequenti causano l'esistenza di molti buffer simultaneamente, consumando memoria
- ⚠️ I periodi di buffering possono sovrapporsi
- ⚠️ Può essere difficile da debuggare a causa dei controlli complessi

## 🚀 Prossimi Passi

- [buffer](/it/guide/operators/transformation/buffer) - Impara il buffering base
- [bufferTime](/it/guide/operators/transformation/bufferTime) - Impara il buffering basato sul tempo
- [bufferCount](/it/guide/operators/transformation/bufferCount) - Impara il buffering basato sul conteggio
- [bufferWhen](/it/guide/operators/transformation/bufferWhen) - Impara il controllo dinamico della fine
- [Casi d'Uso Pratici Operatori di Trasformazione](/it/guide/operators/transformation/practical-use-cases) - Impara casi d'uso reali
