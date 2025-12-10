---
description: "interval() - Funzione di Creazione che emette continuamente valori a intervalli specificati: Essenziale per polling, task periodici, timer countdown e aggiornamenti in tempo reale"
titleTemplate: ':title | RxJS'
---

# interval() - Emissione Continua a Intervalli Specificati

`interval()` è una Funzione di Creazione che emette continuamente valori a intervalli di tempo specificati.

## Panoramica

`interval()` emette continuamente numeri consecutivi a partire da 0 a intervalli di millisecondi specificati. È frequentemente usata per processi di polling ed esecuzione di task periodici.

**Firma**:
```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

**Documentazione Ufficiale**: [📘 RxJS Ufficiale: interval()](https://rxjs.dev/api/index/function/interval)

## Uso Base

`interval()` emette numeri che contano a un intervallo specificato.

```typescript
import { interval } from 'rxjs';

// Emetti valori ogni 1 secondo
const interval$ = interval(1000);

interval$.subscribe(value => {
  console.log('Valore:', value);
});

// Output (ogni 1 secondo):
// Valore: 0
// Valore: 1
// Valore: 2
// Valore: 3
// ... (continua all'infinito)
```

## Caratteristiche Importanti

### 1. Numeri Consecutivi a Partire da 0

`interval()` emette sempre interi che partono da 0 e incrementano di 1.

```typescript
import { interval } from 'rxjs';
import { take } from 'rxjs';

interval(500).pipe(
  take(5) // Ottieni solo i primi 5 valori
).subscribe(value => console.log(value));

// Output (ogni 500ms):
// 0
// 1
// 2
// 3
// 4
```

### 2. Non Completa Mai (Stream Infinito)

`interval()` non completa automaticamente e **deve essere annullato**.

```typescript
import { interval } from 'rxjs';

const subscription = interval(1000).subscribe(value => {
  console.log('Valore:', value);
});

// Annulla subscription dopo 5 secondi
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Fermato');
}, 5000);
```

> [!WARNING]
> **Dimenticare di Annullare Subscription Causa Memory Leak**
>
> Poiché `interval()` continua a emettere valori indefinitamente, dimenticare di annullare può causare memory leak e problemi di performance. Assicurati di chiamare `unsubscribe()` o usa operatori come `take()`, `takeUntil()`, o `takeWhile()` per completare automaticamente.

### 3. Cold Observable

`interval()` è un Cold Observable, che crea un timer indipendente per ogni subscription.

```typescript
import { interval } from 'rxjs';

const interval$ = interval(1000);

// Subscription 1
interval$.subscribe(value => console.log('Observer 1:', value));

// Aggiungi subscription 2 dopo 2 secondi
setTimeout(() => {
  interval$.subscribe(value => console.log('Observer 2:', value));
}, 2000);

// Output:
// Observer 1: 0
// Observer 1: 1
// Observer 2: 0  ← Inizia da 0 con timer indipendente
// Observer 1: 2
// Observer 2: 1
```

> [!NOTE]
> **Caratteristiche Cold Observable**:
> - Un'esecuzione indipendente viene avviata per ogni subscription
> - Ogni subscriber riceve il proprio stream di dati
> - Un timer indipendente viene avviato per ogni subscription; usa `share()` se devi condividere dati
>
> Vedi [Cold Observable e Hot Observable](/it/guide/observables/cold-and-hot-observables) per maggiori informazioni.

## Differenza tra interval() e timer()

Sebbene `interval()` e `timer()` siano simili, ci sono alcune differenze importanti.

```typescript
import { interval, timer } from 'rxjs';
import { take } from 'rxjs';

// interval() - inizia immediatamente, emissione continua
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - inizia dopo ritardo
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// Output:
// interval: 0  (dopo 1 secondo)
// interval: 1  (dopo 2 secondi)
// timer: 0     (dopo 2 secondi)
// interval: 2  (dopo 3 secondi)
// timer: 1     (dopo 3 secondi)
// timer: 2     (dopo 4 secondi)
```

| Funzione di Creazione | Timing di Avvio | Scopo |
|-------------------|--------------|---------|
| `interval(1000)` | Inizia immediatamente (primo valore dopo 1 secondo) | Esecuzione periodica |
| `timer(2000, 1000)` | Inizia dopo tempo specificato | Esecuzione periodica con ritardo |
| `timer(2000)` | Emette solo una volta dopo tempo specificato | Esecuzione ritardata |

## Casi d'Uso Pratici

### 1. Polling API

Chiama API a intervalli regolari per aggiornare dati.

```typescript
import { from, interval } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

function fetchStatus(): Promise<Status> {
  return fetch('https://jsonplaceholder.typicode.com/users/1')
    .then(res => res.json());
}

// Polling API ogni 5 secondi
const polling$ = interval(5000).pipe(
  switchMap(() => from(fetchStatus())),
  catchError(error => {
    console.error('Errore API:', error);
    return of({ status: 'error', timestamp: Date.now() });
  })
);

const subscription = polling$.subscribe(data => {
  console.log('Aggiornamento stato:', data);
});

// Ferma quando necessario
// subscription.unsubscribe();
```

### 2. Timer Countdown

Implementa un countdown per il limite di tempo.

```typescript
import { interval } from 'rxjs';
import { map, takeWhile } from 'rxjs';

const countdown$ = interval(1000).pipe(
  map(count => 10 - count), // Countdown da 10 secondi
  takeWhile(time => time >= 0) // Completa automaticamente a 0
);

countdown$.subscribe({
  next: time => console.log(`Tempo rimanente: ${time} secondi`),
  complete: () => console.log('Tempo scaduto!')
});

// Output (ogni 1 secondo):
// Tempo rimanente: 10 secondi
// Tempo rimanente: 9 secondi
// ...
// Tempo rimanente: 0 secondi
// Tempo scaduto!
```

### 3. Funzione Auto-salvataggio

Salva automaticamente contenuti form periodicamente.

```typescript
import { fromEvent, from } from 'rxjs';
import { switchMap, debounceTime } from 'rxjs';

// Crea form
const form = document.createElement('form');
form.id = 'myForm';
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Inserisci testo';
form.appendChild(input);
document.body.appendChild(form);

const input$ = fromEvent(form, 'input');

// Auto-salva 3 secondi dopo che l'input si ferma (abbreviato per demo)
input$.pipe(
  debounceTime(3000), // Se non c'è input per 3 secondi
  switchMap(() => {
    const formData = new FormData(form);
    // Demo: Simula con Promise invece di API reale
    return from(
      Promise.resolve({ success: true, data: formData.get('text') })
    );
  })
).subscribe(result => {
  console.log('Salvato automaticamente:', result);
});
```

### 4. Visualizzazione Orologio in Tempo Reale

Aggiorna l'ora corrente in tempo reale.

```typescript
import { interval } from 'rxjs';
import { map } from 'rxjs';

// Crea elemento per visualizzazione orologio
const clockElement = document.createElement('div');
clockElement.id = 'clock';
clockElement.style.fontSize = '24px';
clockElement.style.fontFamily = 'monospace';
clockElement.style.padding = '20px';
document.body.appendChild(clockElement);

const clock$ = interval(1000).pipe(
  map(() => new Date().toLocaleTimeString())
);

clock$.subscribe(time => {
  clockElement.textContent = time;
});

// Output: L'ora corrente si aggiorna ogni secondo
```

## Uso in Pipeline

`interval()` viene usato come punto di partenza per pipeline o come trigger di controllo temporale.

```typescript
import { interval } from 'rxjs';
import { map, filter, scan } from 'rxjs';

// Conta solo i secondi pari
interval(1000).pipe(
  filter(count => count % 2 === 0),
  scan((sum, count) => sum + count, 0),
  map(sum => `Somma pari: ${sum}`)
).subscribe(console.log);

// Output (ogni 1 secondo):
// Somma pari: 0
// Somma pari: 2  (0 + 2)
// Somma pari: 6  (0 + 2 + 4)
// Somma pari: 12 (0 + 2 + 4 + 6)
```

## Errori Comuni

### 1. Dimenticare di Annullare Subscription

```typescript
// ❌ Sbagliato - esegue all'infinito senza unsubscribe
import { interval } from 'rxjs';

function startPolling() {
  interval(1000).subscribe(value => {
    console.log('Valore:', value); // Esegue per sempre
  });
}

startPolling();

// ✅ Corretto - mantieni subscription e annulla quando necessario
import { interval, Subscription } from 'rxjs';

let subscription: Subscription | null = null;

function startPolling() {
  subscription = interval(1000).subscribe(value => {
    console.log('Valore:', value);
  });
}

function stopPolling() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startPolling();
// Chiama stopPolling() quando necessario
```

### 2. Subscription Multiple Creano Timer Indipendenti

```typescript
// ❌ Non intenzionale - vengono creati due timer indipendenti
import { interval } from 'rxjs';

const interval$ = interval(1000);

interval$.subscribe(value => console.log('Observer 1:', value));
interval$.subscribe(value => console.log('Observer 2:', value));
// Due timer eseguono in parallelo

// ✅ Corretto - condividi un timer
import { interval } from 'rxjs';
import { share } from 'rxjs';

const interval$ = interval(1000).pipe(share());

interval$.subscribe(value => console.log('Observer 1:', value));
interval$.subscribe(value => console.log('Observer 2:', value));
// Un timer viene condiviso
```

## Considerazioni sulle Performance

Sebbene `interval()` sia leggero, le performance dovrebbero essere considerate quando si esegue a intervalli brevi.

> [!TIP]
> **Suggerimenti di Ottimizzazione**:
> - Non eseguire elaborazioni non necessarie (raffina con `filter()`)
> - Usa intervalli brevi (meno di 100ms) con cautela
> - Assicurati che le subscription vengano annullate
> - Se servono Observer multipli, condividili con `share()`

```typescript
import { interval } from 'rxjs';
import { filter, share } from 'rxjs';

// ❌ Problema di performance - elaborazione pesante ogni 100ms
interval(100).subscribe(() => {
  // Elaborazione pesante
  heavyCalculation();
});

// ✅ Ottimizzazione - elabora solo quando necessario
interval(100).pipe(
  filter(count => count % 10 === 0), // Una volta al secondo (una volta ogni 10)
  share() // Condividi tra Observer multipli
).subscribe(() => {
  heavyCalculation();
});
```

## Funzioni di Creazione Correlate

| Funzione | Differenza | Uso |
|----------|------|----------|
| **[timer()](/it/guide/creation-functions/basic/timer)** | Inizia dopo ritardo, o emette solo una volta | Esecuzione ritardata o elaborazione una tantum |
| **[fromEvent()](/it/guide/creation-functions/basic/fromEvent)** | Guidato da eventi | Elaborazione in base a operazioni utente |
| **range()** | Emette numeri in range specificato immediatamente | Quando il controllo temporale non serve |

## Riepilogo

- `interval()` emette continuamente valori a intervalli specificati
- Emette interi consecutivi a partire da 0
- Non completa automaticamente, deve essere annullato
- Funziona come Cold Observable (timer indipendente per ogni subscription)
- Ideale per polling, esecuzione periodica, countdown, ecc.

## Prossimi Passi

- [timer() - Inizia a Emettere Dopo un Ritardo](/it/guide/creation-functions/basic/timer)
- [fromEvent() - Converti Eventi in Observable](/it/guide/creation-functions/basic/fromEvent)
- [Torna alle Funzioni di Creazione Base](/it/guide/creation-functions/basic/)
