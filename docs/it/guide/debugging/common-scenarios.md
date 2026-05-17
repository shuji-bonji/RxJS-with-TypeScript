---
description: "6 scenari comuni: nessun valore, valori errati, subscription infinita, memory leak, errori ignorati. Soluzioni pratiche per debug RxJS."
---

# Scenari di Debug Comuni

Spiegazione dei problemi tipici incontrati nello sviluppo RxJS e delle loro soluzioni, con esempi di codice concreti.

## Scenario 1: Nessun Valore Emesso

- **Sintomo**: Nonostante la `subscribe`, non viene emesso nessun valore

### Causa 1: Subscription Dimenticata di Cold Observable

```ts
import { interval } from 'rxjs';
import { map } from 'rxjs';

// ❌ Non viene eseguito nulla perché non c'è subscription
const numbers$ = interval(1000).pipe(
  map(x => {
    console.log('Questa riga non viene eseguita');
    return x * 2;
  })
);

// ✅ Viene eseguito con la subscription
numbers$.subscribe(value => console.log('Valore:', value));
```

### Causa 2: Subject Già Completato

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.complete(); // Completamento

// ❌ La subscription dopo il completamento non riceve valori
subject.subscribe(value => console.log('Questa riga non viene eseguita'));

// ✅ Subscribe prima del completamento
const subject2 = new Subject<number>();
subject2.subscribe(value => console.log('Valore:', value));
subject2.next(1); // Valore: 1
subject2.complete();
```

### Causa 3: Filtraggio con Condizioni Errate

```ts
import { of } from 'rxjs';
import { filter, tap } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('Prima di filter:', value)),
    filter(x => x > 10), // Tutti vengono esclusi
    tap(value => console.log('Dopo filter:', value)) // Questa riga non viene eseguita
  )
  .subscribe({
    next: value => console.log('Valore finale:', value),
    complete: () => console.log('Completato (nessun valore)')
  });

// Output:
// Prima di filter: 1
// Prima di filter: 2
// Prima di filter: 3
// Prima di filter: 4
// Prima di filter: 5
// Completato (nessun valore)
```

### Tecnica di Debug
```ts
import { of, EMPTY } from 'rxjs';
import { filter, tap, defaultIfEmpty } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('🔵 Input:', value)),
    filter(x => x > 10),
    tap(value => console.log('🟢 Passato filter:', value)),
    defaultIfEmpty('Nessun valore') // Valore predefinito quando non ci sono valori
  )
  .subscribe(value => console.log('✅ Output:', value));

// Output:
// 🔵 Input: 1
// 🔵 Input: 2
// 🔵 Input: 3
// 🔵 Input: 4
// 🔵 Input: 5
// ✅ Output: Nessun valore
```

## Scenario 2: Valori Diversi da Quelli Attesi

- **Sintomo**: Vengono emessi valori diversi da quelli previsti

### Causa 1: Ordine Errato degli Operatori

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// ❌ Risultato diverso da quello atteso
of(1, 2, 3, 4, 5)
  .pipe(
    map(x => x * 2),     // 2, 4, 6, 8, 10
    filter(x => x < 5)   // Passano solo 2, 4
  )
  .subscribe(value => console.log('Risultato:', value));
// Output: 2, 4

// ✅ Ordine corretto
of(1, 2, 3, 4, 5)
  .pipe(
    filter(x => x < 5),  // Passano solo 1, 2, 3, 4
    map(x => x * 2)      // 2, 4, 6, 8
  )
  .subscribe(value => console.log('Risultato:', value));
// Output: 2, 4, 6, 8
```

### Causa 2: Modifica Non Intenzionale per Condivisione Riferimento

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const user: User = { id: 1, name: 'Alice' };

of(user)
  .pipe(
    // ❌ Modifica direttamente l'oggetto originale
    map(u => {
      u.name = 'Bob'; // L'oggetto originale viene modificato
      return u;
    })
  )
  .subscribe(value => console.log('Dopo modifica:', value));

console.log('Oggetto originale:', user); // { id: 1, name: 'Bob' }

// ✅ Creare un nuovo oggetto
of(user)
  .pipe(
    map(u => ({ ...u, name: 'Charlie' })) // Nuovo oggetto con spread syntax
  )
  .subscribe(value => console.log('Dopo modifica:', value));

console.log('Oggetto originale:', user); // { id: 1, name: 'Alice' } (non modificato)
```

### Causa 3: Timing di Elaborazione Asincrona

```ts
import { of, delay } from 'rxjs';
import { mergeMap, tap } from 'rxjs';

// ❌ Non attende il completamento dell'elaborazione asincrona
of(1, 2, 3)
  .pipe(
    tap(value => console.log('Inizio:', value)),
    mergeMap(value =>
      of(value * 2).pipe(
        delay(100 - value * 10) // Più grande è il valore, più veloce il completamento
      )
    )
  )
  .subscribe(value => console.log('Completato:', value));

// Output:
// Inizio: 1
// Inizio: 2
// Inizio: 3
// Completato: 3  ← Ritardo più breve
// Completato: 2
// Completato: 1  ← Ritardo più lungo

// ✅ Garantire l'ordine
import { concatMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(value => console.log('Inizio:', value)),
    concatMap(value =>  // mergeMap → concatMap
      of(value * 2).pipe(delay(100 - value * 10))
    )
  )
  .subscribe(value => console.log('Completato:', value));

// Output:
// Inizio: 1
// Completato: 1
// Inizio: 2
// Completato: 2
// Inizio: 3
// Completato: 3
```

## Scenario 3: Subscription Non Completata (Stream Infinito)

- **Sintomo**: `complete` non viene chiamato e lo stream non termina

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

// ❌ interval continua ad emettere valori all'infinito
interval(1000)
  .pipe(
    tap(value => console.log('Valore:', value))
  )
  .subscribe({
    complete: () => console.log('Questa riga non viene eseguita')
  });

// ✅ Completare esplicitamente con take
import { take } from 'rxjs';

interval(1000)
  .pipe(
    take(5), // Completa dopo 5 valori
    tap(value => console.log('Valore:', value))
  )
  .subscribe({
    complete: () => console.log('Completato')
  });
```

### Tecnica di Debug
```ts
import { interval, timer } from 'rxjs';
import { tap, takeUntil } from 'rxjs';

// Debug con timeout impostato
const stop$ = timer(5000); // Completa dopo 5 secondi

interval(1000)
  .pipe(
    takeUntil(stop$),
    tap({
      next: value => console.log('Valore:', value),
      complete: () => console.log('Fermato per timeout')
    })
  )
  .subscribe();
```

## Scenario 4: Memory Leak (Subscription Non Cancellata)

- **Sintomo**: L'applicazione diventa gradualmente più lenta

### Causa: Subscription Non Necessaria Non Cancellata

```ts
import { interval } from 'rxjs';

class UserComponent {
  private subscription: any;

  ngOnInit() {
    // ❌ Subscription dimenticata
    interval(1000).subscribe(value => {
      console.log('Valore:', value); // Continua ad essere eseguito anche dopo la distruzione del componente
    });
  }

  ngOnDestroy() {
    // La subscription non viene cancellata
  }
}

// ✅ Gestione appropriata della subscription
class UserComponentFixed {
  private subscription: any;

  ngOnInit() {
    this.subscription = interval(1000).subscribe(value => {
      console.log('Valore:', value);
    });
  }

  ngOnDestroy() {
    // Cancellare la subscription alla distruzione del componente
    if (this.subscription) {
      this.subscription.unsubscribe();
    }
  }
}
```

**Pattern Consigliato: Usare `takeUntil`**

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class UserComponentBest {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    // ✅ Cancellazione automatica con takeUntil
    interval(1000)
      .pipe(
        takeUntil(this.destroy$)
      )
      .subscribe(value => console.log('Valore:', value));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### Rilevamento Memory Leak

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

let subscriptionCount = 0;

const trackSubscriptions = <T>() =>
  tap<T>({
    subscribe: () => {
      subscriptionCount++;
      console.log('📈 Numero subscription:', subscriptionCount);
    },
    unsubscribe: () => {
      subscriptionCount--;
      console.log('📉 Numero subscription:', subscriptionCount);
    }
  });

// Esempio d'uso
const stream$ = interval(1000).pipe(
  trackSubscriptions()
);

const sub1 = stream$.subscribe();
// Output: 📈 Numero subscription: 1

const sub2 = stream$.subscribe();
// Output: 📈 Numero subscription: 2

setTimeout(() => {
  sub1.unsubscribe();
  // Output: 📉 Numero subscription: 1
}, 3000);
```

## Scenario 5: Errori Non Rilevati

- **Sintomo**: Gli errori vengono ignorati senza essere visualizzati

```ts
import { of, throwError } from 'rxjs';
import { mergeMap, catchError } from 'rxjs';

// ❌ Gli errori vengono soppressi perché non c'è gestione degli errori
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Errore'));
      }
      return of(value);
    })
  )
  .subscribe(); // Nessun error handler

// ✅ Gestione appropriata degli errori
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Errore'));
      }
      return of(value);
    }),
    catchError((error: unknown) => {
      console.error('🔴 Errore catturato:', (error instanceof Error ? error.message : String(error)));
      return of(-1); // Valore di fallback
    })
  )
  .subscribe({
    next: value => console.log('Valore:', value),
    error: error => console.error('🔴 Errore nella subscribe:', error)
  });

// Output:
// Valore: 1
// 🔴 Errore catturato: Errore
// Valore: -1
```

### Configurazione Error Handler Globale

```ts
import { Observable } from 'rxjs';

// Catturare tutti gli errori non gestiti
const originalCreate = Observable.create;

Observable.create = function(subscribe: any) {
  return originalCreate.call(this, (observer: any) => {
    try {
      return subscribe(observer);
    } catch (error) {
      console.error('🔴 Errore non gestito:', error);
      observer.error(error);
    }
  });
};
```

## Scenario 6: Tracciamento Tentativi di Retry

- **Sintomo**: Quando si usa l'operatore `retry`, non si sa quanti tentativi vengono effettuati

Quando si effettua il retry automaticamente in caso di errore, tracciare effettivamente quanti tentativi vengono eseguiti facilita il debug e la registrazione dei log.

### Debug Base del Retry

```ts
import { throwError, of, timer } from 'rxjs';
import { retryWhen, mergeMap, tap } from 'rxjs';

throwError(() => new Error('Errore temporaneo'))
  .pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          console.log(`🔄 Tentativo ${retryCount}`);

          if (retryCount > 2) {
            console.log('❌ Raggiunto numero massimo tentativi');
            throw error;
          }

          return timer(1000);
        })
      )
    )
  )
  .subscribe({
    next: value => console.log('✅ Successo:', value),
    error: error => console.log('🔴 Errore finale:', error.message)
  });

// Output:
// 🔄 Tentativo 1
// 🔄 Tentativo 2
// 🔄 Tentativo 3
// ❌ Raggiunto numero massimo tentativi
// 🔴 Errore finale: Errore temporaneo
```

> [!TIP]
> Per pattern di implementazione più dettagliati sui metodi di debug del retry, consultare la sezione "Debug del Retry" in [retry e catchError](/it/guide/error-handling/retry-catch#debug-del-retry).
> - Tracciamento base utilizzando error callback di tap
> - Registrazione dettagliata dei log con retryWhen
> - Exponential backoff e registrazione log
> - Oggetto di configurazione retry in RxJS 7.4+

## Riepilogo

Soluzioni per scenari di debug comuni

- ✅ **Nessun valore emesso** → Verificare subscription dimenticata, condizioni di filtro
- ✅ **Valori diversi da attesi** → Attenzione all'ordine degli operatori, condivisione riferimenti
- ✅ **Subscription non completata** → Usare `take` o `takeUntil` per stream infiniti
- ✅ **Memory leak** → Pattern `takeUntil` per cancellazione automatica subscription
- ✅ **Errori ignorati** → Implementare gestione errori appropriata
- ✅ **Tracciamento retry** → Registrazione log con `retryWhen` o oggetto di configurazione

## Pagine Correlate

- [Strategie di Base per il Debug](/it/guide/debugging/) - Come usare operatore tap e strumenti per sviluppatori
- [Strumenti di Debug Personalizzati](/it/guide/debugging/custom-tools) - Stream nominati, operatori di debug
- [Debug delle Prestazioni](/it/guide/debugging/performance) - Monitoraggio subscription, verifica uso memoria
- [Gestione degli Errori](/it/guide/error-handling/strategies) - Strategie di gestione errori
