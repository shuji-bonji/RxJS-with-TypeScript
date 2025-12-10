---
description: "Tecniche di debug RxJS: tap(), console.log, DevTools, operatori personalizzati e misurazione prestazioni per applicazioni reattive efficienti."
---

# Tecniche di Debug RxJS

Il debug di RxJS richiede un approccio diverso rispetto alle tradizionali tecniche di debug sincrone, a causa della natura asincrona degli stream.

Questa pagina fornisce strategie di base per il debug delle applicazioni RxJS e la navigazione verso tecniche di debug più dettagliate.

## Panoramica delle Tecniche di Debug

Il debug di RxJS può essere classificato in 4 approcci principali.

| Approccio | Contenuto | Pagina Dettagliata |
|----------|------|-----------|
| **Strategie di Base** | Operatore tap, strumenti per sviluppatori, RxJS DevTools | Spiegato in questa pagina |
| **Scenari Comuni** | 6 problemi tipici: nessun valore emesso, memory leak, errori ignorati | [→ Dettagli](/it/guide/debugging/common-scenarios) |
| **Strumenti Personalizzati** | Stream nominati, operatori di debug, misurazione delle prestazioni | [→ Dettagli](/it/guide/debugging/custom-tools) |
| **Prestazioni** | Monitoraggio delle subscription, rilevamento ricalcoli, uso memoria | [→ Dettagli](/it/guide/debugging/performance) |

## Strategie di Base per il Debug

### 1. Log con l'Operatore `tap`

L'operatore `tap` è la tecnica di debug più basilare che permette di osservare i valori dello stream senza effetti collaterali.

```ts
import { interval } from 'rxjs';
import { map, filter, tap } from 'rxjs';

interval(1000)
  .pipe(
    tap(value => console.log('🔵 Valore originale:', value)),
    map(x => x * 2),
    tap(value => console.log('🟢 Dopo map:', value)),
    filter(x => x > 5),
    tap(value => console.log('🟡 Dopo filter:', value))
  )
  .subscribe(value => console.log('✅ Valore finale:', value));

// Output:
// 🔵 Valore originale: 0
// 🟢 Dopo map: 0
// 🔵 Valore originale: 1
// 🟢 Dopo map: 2
// 🔵 Valore originale: 2
// 🟢 Dopo map: 4
// 🔵 Valore originale: 3
// 🟢 Dopo map: 6
// 🟡 Dopo filter: 6
// ✅ Valore finale: 6
```

#### Punti Chiave
- Inserendo `tap` ad ogni step della pipeline, è possibile tracciare il flusso dei dati
- Usare emoji o etichette migliora la leggibilità dei log
- `tap` non modifica i valori, quindi è sicuro inserire log di debug

### 2. Output di Informazioni Dettagliate

Per ottenere informazioni di debug più dettagliate, utilizzare l'oggetto Observer.

```ts
import { of, throwError, concat } from 'rxjs';
import { tap } from 'rxjs';

const debug = (tag: string) =>
  tap({
    next: value => console.log(`[${tag}] next:`, value),
    error: error => console.error(`[${tag}] error:`, error),
    complete: () => console.log(`[${tag}] complete`)
  });

// Stream normale
of(1, 2, 3)
  .pipe(debug('Normale'))
  .subscribe();

// Output:
// [Normale] next: 1
// [Normale] next: 2
// [Normale] next: 3
// [Normale] complete

// Stream con errore
concat(
  of(1, 2),
  throwError(() => new Error('Errore verificato'))
)
  .pipe(debug('Errore'))
  .subscribe({
    error: () => {} // Gestione errori
  });

// Output:
// [Errore] next: 1
// [Errore] next: 2
// [Errore] error: Error: Errore verificato
```

### 3. Utilizzo degli Strumenti per Sviluppatori

Tecniche di debug utilizzando gli strumenti per sviluppatori del browser.

```ts
import { fromEvent, timer } from 'rxjs';
import { map, tap, debounceTime } from 'rxjs';

// Funzione helper per il debug
function tapDebugger<T>(label: string) {
  return tap<T>({
    next: value => {
      console.group(`🔍 ${label}`);
      console.log('Value:', value);
      console.log('Type:', typeof value);
      console.log('Timestamp:', new Date().toISOString());
      console.trace('Stack trace');
      console.groupEnd();
    }
  });
}

// Debug evento click pulsante
const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tapDebugger('Click Event'),
      debounceTime(300),
      tapDebugger('After Debounce'),
      map(() => ({ timestamp: Date.now() }))
    )
    .subscribe(data => console.log('📤 Invio:', data));
}
```

#### Utilizzo degli Strumenti per Sviluppatori
- Raggruppare i log con `console.group()`
- Visualizzare lo stack trace con `console.trace()`
- Visualizzare array e oggetti in modo leggibile con `console.table()`
- Impostare breakpoint all'interno di `tap`

### 4. Utilizzo di RxJS DevTools

RxJS DevTools è uno strumento di debug fornito come estensione del browser.

#### Installazione
- Chrome: [RxJS DevTools - Chrome Web Store](https://chrome.google.com/webstore)
- Firefox: [RxJS DevTools - Firefox Add-ons](https://addons.mozilla.org/)

#### Funzionalità Principali
- Visualizzazione dello stato delle subscription Observable
- Visualizzazione timeline dei valori dello stream
- Rilevamento memory leak
- Analisi delle prestazioni

#### Esempio d'Uso

```ts
import { interval } from 'rxjs';
import { take, map } from 'rxjs';

// Abilitare il debug solo in ambiente di sviluppo
// Il metodo di verifica della variabile d'ambiente varia in base al build tool
const isDevelopment =
  // Vite: import.meta.env.DEV
  // webpack: process.env.NODE_ENV === 'development'
  // Configurazione manuale: usare variabile globale
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

const stream$ = interval(1000).pipe(
  take(5),
  map(x => x * 2)
);

if (isDevelopment) {
  // Rendere osservabile con DevTools
  stream$.subscribe({
    next: value => console.log('DevTools:', value)
  });
}
```

## Tecniche di Debug Dettagliate

Dopo aver compreso le strategie di base, apprendere tecniche di debug specifiche nelle seguenti pagine dettagliate.

### Scenari di Debug Comuni

6 problemi tipici incontrati nello sviluppo reale e le loro soluzioni

- Scenario 1: Nessun valore emesso
- Scenario 2: Valori diversi da quelli attesi
- Scenario 3: Subscription non completata (stream infinito)
- Scenario 4: Memory leak (subscription non cancellata)
- Scenario 5: Errori non rilevati
- Scenario 6: Tracciamento tentativi di retry

[→ Vedi Scenari di Debug Comuni](/it/guide/debugging/common-scenarios)

### Strumenti di Debug Personalizzati

Come creare strumenti di debug personalizzati per i requisiti del progetto

- Debug di stream nominati (tagStream)
- Creazione di operatori di debug personalizzati
- Operatore di misurazione delle prestazioni (measure)

[→ Vedi Strumenti di Debug Personalizzati](/it/guide/debugging/custom-tools)

### Debug delle Prestazioni

Ottimizzazione delle applicazioni e best practice

- Verifica e tracciamento del numero di subscription
- Rilevamento di ricalcoli non necessari (shareReplay)
- Monitoraggio dell'uso della memoria
- Configurazione dell'ambiente di debug
- Debug type-safe
- Impostazione di error boundary

[→ Vedi Debug delle Prestazioni](/it/guide/debugging/performance)

## Riepilogo

Il debug di RxJS può essere eseguito in modo efficiente tenendo conto dei seguenti punti.

### Strategie di Base
- ✅ Osservare ogni fase dello stream con l'operatore `tap`
- ✅ Output dettagliato dei log utilizzando gli strumenti per sviluppatori
- ✅ Visualizzare gli stream con RxJS DevTools

### Scenari Comuni
- ✅ Nessun valore emesso → Verificare subscription dimenticata, condizioni di filtro
- ✅ Valori diversi da attesi → Attenzione all'ordine degli operatori, condivisione riferimenti
- ✅ Subscription non completata → Usare `take` o `takeUntil` per stream infiniti
- ✅ Memory leak → Pattern `takeUntil` per cancellazione automatica subscription
- ✅ Errori ignorati → Implementare gestione errori appropriata

### Strumenti di Debug
- ✅ Debug flessibile con operatori di debug personalizzati
- ✅ Tracciare multipli stream con stream nominati
- ✅ Identificare colli di bottiglia con misurazione delle prestazioni

### Prestazioni
- ✅ Prevenire memory leak monitorando il numero di subscription
- ✅ Evitare ricalcoli non necessari con `shareReplay`
- ✅ Verificare periodicamente l'uso della memoria

Combinando queste tecniche, è possibile effettuare il debug delle applicazioni RxJS in modo efficiente.

## Pagine Correlate

- [Gestione degli Errori](/it/guide/error-handling/strategies) - Strategie di gestione degli errori
- [Tecniche di Test](/it/guide/testing/unit-tests) - Metodi di test RxJS
- [Anti-pattern RxJS](/it/guide/anti-patterns/) - Errori comuni e contromisure
- [Pipeline](/it/guide/operators/pipeline) - Concatenazione degli operatori
