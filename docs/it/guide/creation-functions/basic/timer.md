---
description: "timer() - Funzione di Creazione che inizia a emettere dopo un ritardo specificato: Perfetto per esecuzione ritardata, polling con ritardo e implementazioni di timeout"
---

# timer() - Inizia a Emettere Dopo un Ritardo

`timer()` è una Funzione di Creazione che inizia a emettere valori dopo un tempo di ritardo specificato, supportando sia emissione singola che periodica.

## Panoramica

`timer()` è una Funzione di Creazione flessibile che ti permette di controllare il timing della prima emissione. Il suo comportamento cambia in base al numero di argomenti, e può essere usato sia per emissione singola che periodica come `interval()`.

**Firma**:
```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

**Documentazione Ufficiale**: [📘 RxJS Ufficiale: timer()](https://rxjs.dev/api/index/function/timer)

## Uso Base

Il comportamento di `timer()` dipende dal numero di argomenti.

### Emissione Singola

Se viene specificato solo il primo argomento, emette 0 dopo il tempo specificato e completa.

```typescript
import { timer } from 'rxjs';

// Emetti 0 dopo 3 secondi e completa
const timer$ = timer(3000);

timer$.subscribe({
  next: value => console.log('Valore:', value),
  complete: () => console.log('Completo')
});

// Output dopo 3 secondi:
// Valore: 0
// Completo
```

### Emissione Periodica

Se viene specificato un intervallo per il secondo argomento, continuerà a emettere periodicamente dopo il ritardo iniziale.

```typescript
import { timer } from 'rxjs';

// Inizia dopo 3 secondi, poi emetti valori ogni 1 secondo
const timer$ = timer(3000, 1000);

timer$.subscribe(value => console.log('Valore:', value));

// Output:
// Valore: 0  (dopo 3 secondi)
// Valore: 1  (dopo 4 secondi)
// Valore: 2  (dopo 5 secondi)
// ... (continua all'infinito)
```

## Caratteristiche Importanti

### 1. Specifica Flessibile dei Ritardi

Il ritardo può essere specificato come numero in millisecondi o come oggetto `Date`.

```typescript
import { timer } from 'rxjs';

// Specifica in millisecondi
timer(5000).subscribe(() => console.log('Dopo 5 secondi'));

// Specifica con oggetto Date (esegui a tempo specifico)
const targetTime = new Date(Date.now() + 10000); // 10 secondi dopo
timer(targetTime).subscribe(() => console.log('Esegui a tempo specificato'));
```

### 2. Il Comportamento Cambia in Base al Secondo Argomento

Se il secondo argomento è specificato o meno determina se completa.

```typescript
import { timer } from 'rxjs';

// Senza secondo argomento - emette una volta e completa
timer(1000).subscribe({
  next: value => console.log('Una volta:', value),
  complete: () => console.log('Completo')
});

// Con secondo argomento - emette all'infinito
timer(1000, 1000).subscribe({
  next: value => console.log('Ripeti:', value),
  complete: () => console.log('Completo (non visualizzato)')
});
```

> [!IMPORTANT]
> **Con Secondo Argomento, Non Completa**
>
> Se specifichi il secondo argomento come `timer(1000, 1000)`, continuerà a emettere indefinitamente, proprio come `interval()`. L'annullamento è sempre richiesto.

### 3. Cold Observable

`timer()` è un Cold Observable, il che significa che viene creato un timer indipendente per ogni subscription.

```typescript
import { timer } from 'rxjs';

const timer$ = timer(1000);

console.log('Inizio');

// Subscription 1
timer$.subscribe(() => console.log('Observer 1'));

// Aggiungi subscription 2 dopo 500ms
setTimeout(() => {
  timer$.subscribe(() => console.log('Observer 2'));
}, 500);

// Output:
// Inizio
// Observer 1  (dopo 1 secondo)
// Observer 2  (dopo 1.5 secondi - timer indipendente)
```

> [!NOTE]
> **Caratteristiche Cold Observable**:
> - Un'esecuzione indipendente viene avviata per ogni subscription
> - Ogni subscriber riceve il proprio stream di dati
> - Un timer indipendente viene avviato per ogni subscription; come con `interval()`, usa `share()` se serve condivisione
>
> Vedi [Cold Observable e Hot Observable](/it/guide/observables/cold-and-hot-observables) per maggiori informazioni.

## Differenza tra timer() e interval()

La differenza principale tra i due è il timing della prima emissione.

```typescript
import { timer, interval } from 'rxjs';
import { take } from 'rxjs';

console.log('Inizio');

// interval() - inizia immediatamente (primo valore dopo 1 secondo)
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - nessun ritardo (primo valore immediatamente)
timer(0, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// timer() - inizia dopo ritardo di 2 secondi
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer(delay):', value);
});
```

| Funzione di Creazione | Timing Prima Emissione | Scopo |
|-------------------|----------------------|---------|
| `interval(1000)` | Dopo 1 secondo | Inizia esecuzione periodica immediatamente |
| `timer(0, 1000)` | Immediatamente | Vuoi prima esecuzione immediatamente |
| `timer(2000, 1000)` | Dopo 2 secondi | Esecuzione periodica dopo ritardo |
| `timer(2000)` | Dopo 2 secondi (una volta sola) | Esecuzione ritardata (una tantum) |

## Casi d'Uso Pratici

### 1. Esecuzione Ritardata

Esegui un processo solo una volta dopo un certo periodo di tempo.

```typescript
import { from, timer } from 'rxjs';
import { switchMap } from 'rxjs';

function delayedApiCall() {
  return timer(2000).pipe(
    switchMap(() => from(
      fetch('https://jsonplaceholder.typicode.com/posts/1')
        .then(res => res.json())
    ))
  );
}

delayedApiCall().subscribe(data => {
  console.log('Recupera dati dopo 2 secondi:', data);
});
```

### 2. Polling con Ritardo

Inizia il polling dopo un certo periodo di tempo invece di eseguire immediatamente la prima volta.

```typescript
import { from, timer } from 'rxjs';
import { switchMap, retry } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

// Inizia polling dopo 5 secondi, poi ogni 10 secondi
const polling$ = timer(5000, 10000).pipe(
  switchMap(() => from(
    fetch('https://jsonplaceholder.typicode.com/users/1')
      .then(res => res.json() as Promise<Status>)
  )),
  retry(3) // Riprova fino a 3 volte in caso di errore
);

const subscription = polling$.subscribe(data => {
  console.log('Aggiornamento stato:', data);
});

// Ferma quando necessario
// subscription.unsubscribe();
```

### 3. Elaborazione Timeout

Il timeout si verifica quando l'elaborazione non viene completata entro un certo periodo di tempo.

```typescript
import { timer, race, from } from 'rxjs';
import { map } from 'rxjs';

function fetchWithTimeout(url: string, timeoutMs: number) {
  const request$ = from(fetch(url).then(res => res.json()));
  const timeout$ = timer(timeoutMs).pipe(
    map(() => {
      throw new Error('Timeout');
    })
  );

  // Usa quello che arriva prima
  return race(request$, timeout$);
}

fetchWithTimeout('https://jsonplaceholder.typicode.com/posts/1', 3000).subscribe({
  next: data => console.log('Recupera dati:', data),
  error: err => console.error('Errore:', err.message)
});
```

### 4. Nascondere Notifiche Automaticamente

Nascondi automaticamente le notifiche dopo un certo periodo di tempo dalla visualizzazione.

```typescript
import { timer, Subject, map } from 'rxjs';
import { switchMap, takeUntil } from 'rxjs';

interface Notification {
  id: number;
  message: string;
}

const notifications$ = new Subject<Notification>();
const dismiss$ = new Subject<number>();

notifications$.pipe(
  switchMap(notification => {
    console.log('Mostra notifica:', notification.message);

    // Nascondi automaticamente dopo 5 secondi
    return timer(5000).pipe(
      takeUntil(dismiss$), // Annulla se dismesso manualmente
      map(() => notification.id)
    );
  })
).subscribe(id => {
  console.log('Nascondi notifica:', id);
});

// Mostra notifica
notifications$.next({ id: 1, message: 'Nuovo messaggio ricevuto' });

// Per dismettere manualmente
// dismiss$.next(1);
```

## Uso in Pipeline

`timer()` viene usato come punto di partenza per elaborazione ritardata o esecuzione periodica.

```typescript
import { timer } from 'rxjs';
import { map, take, scan } from 'rxjs';

// Timer countdown (da 10 secondi a 0 secondi)
timer(0, 1000).pipe(
  map(count => 10 - count),
  take(11), // Da 0 a 10 (11 valori)
  scan((acc, curr) => curr, 0)
).subscribe({
  next: time => console.log(`Rimanente: ${time} secondi`),
  complete: () => console.log('Timer terminato')
});

// Output:
// Rimanente: 10 secondi
// Rimanente: 9 secondi
// ...
// Rimanente: 0 secondi
// Timer terminato
```

## Errori Comuni

### 1. Dimenticare di Annullare Subscription con Secondo Argomento

```typescript
// ❌ Sbagliato - esegue all'infinito con secondo argomento
import { timer } from 'rxjs';

function startTimer() {
  timer(1000, 1000).subscribe(value => {
    console.log('Valore:', value); // Esegue per sempre
  });
}

startTimer();

// ✅ Corretto - mantieni subscription e annulla quando necessario
import { timer, Subscription } from 'rxjs';
import { take } from 'rxjs';

let subscription: Subscription | null = null;

function startTimer() {
  subscription = timer(1000, 1000).pipe(
    take(10) // Completa automaticamente dopo 10 volte
  ).subscribe(value => {
    console.log('Valore:', value);
  });
}

function stopTimer() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startTimer();
```

### 2. Non Comprendere la Differenza da interval()

```typescript
// ❌ Confusione - interval() inizia immediatamente (primo valore dopo 1 secondo)
import { interval } from 'rxjs';

interval(1000).subscribe(value => {
  console.log('interval:', value); // 0 emesso dopo 1 secondo
});

// ✅ timer() - quando vuoi emettere primo valore immediatamente senza ritardo
import { timer } from 'rxjs';

timer(0, 1000).subscribe(value => {
  console.log('timer:', value); // 0 emesso immediatamente
});
```

## Considerazioni sulle Performance

Sebbene `timer()` sia leggero, il suo utilizzo può influenzare le performance.

> [!TIP]
> **Suggerimenti di Ottimizzazione**:
> - Non specificare secondo argomento per esecuzione singola
> - Annulla sempre quando non più necessario
> - Se servono Observer multipli, condividili con `share()`
> - Usa intervalli brevi (meno di 100ms) con cautela

```typescript
import { timer } from 'rxjs';
import { share } from 'rxjs';

// ❌ Problema di performance - timer indipendenti multipli
const timer$ = timer(0, 1000);

timer$.subscribe(value => console.log('Observer 1:', value));
timer$.subscribe(value => console.log('Observer 2:', value));
// Due timer eseguono in parallelo

// ✅ Ottimizzazione - condividi un timer
const sharedTimer$ = timer(0, 1000).pipe(share());

sharedTimer$.subscribe(value => console.log('Observer 1:', value));
sharedTimer$.subscribe(value => console.log('Observer 2:', value));
// Un timer viene condiviso
```

## Funzioni di Creazione Correlate

| Funzione | Differenza | Uso |
|----------|------|----------|
| **[interval()](/it/guide/creation-functions/basic/interval)** | Inizia immediatamente (nessun ritardo) | Esecuzione periodica senza ritardo |
| **[of()](/it/guide/creation-functions/basic/of)** | Emetti sincronamente e immediatamente | Quando asincrono non serve |
| **defer()** | Ritarda elaborazione fino a subscription | Generazione dinamica valori |

## Riepilogo

- `timer()` è una Funzione di Creazione che inizia a emettere dopo un ritardo
- Senza secondo argomento: emissione singola (completa)
- Con secondo argomento: emissione periodica (non completa)
- Il tempo di ritardo può essere specificato in millisecondi o come oggetto `Date`
- Ideale per esecuzione ritardata, polling con ritardo, elaborazione timeout

## Prossimi Passi

- [interval() - Emissione Continua a Intervalli Specificati](/it/guide/creation-functions/basic/interval)
- [defer() - Ritarda la Generazione Fino alla Subscription](/it/guide/creation-functions/conditional/defer)
- [Torna alle Funzioni di Creazione Base](/it/guide/creation-functions/basic/)
