---
description: L'operatore ignoreElements è un operatore di filtraggio RxJS che ignora tutti i valori e fa passare solo le notifiche di completamento ed errore. È utile quando si attende il completamento di un processo.
titleTemplate: ':title'
---

# ignoreElements - Ignora Tutti i Valori e Fa Passare Solo Completamento/Errore

L'operatore `ignoreElements` ignora **tutti i valori** emessi dall'Observable sorgente e fa passare **solo le notifiche di completamento ed errore** a valle.

## 🔰 Sintassi e Utilizzo Base

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valore:', value), // Non chiamato
  complete: () => console.log('Completato')
});
// Output: Completato
```

**Flusso di operazione**:
1. 1, 2, 3, 4, 5 sono tutti ignorati
2. Solo la notifica di completamento viene propagata a valle

[🌐 Documentazione Ufficiale RxJS - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Pattern di Utilizzo Tipici

- **Attesa completamento processo**: Quando i valori non servono e serve solo il completamento
- **Esegui solo side effect**: Esegui side effect con tap e ignora i valori
- **Gestione errori**: Quando vuoi catturare solo gli errori
- **Sincronizzazione sequenze**: Attendi il completamento di più processi

## 🧠 Esempio di Codice Pratico 1: Attesa Completamento Inizializzazione

Esempio di attesa del completamento di più processi di inizializzazione.

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs';

// Crea UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Inizializzazione Applicazione';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// Funzione per aggiungere log di stato
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// Processo inizializzazione 1: Connessione database
const initDatabase$ = from(['Connessione DB...', 'Verifica tabelle...', 'DB pronto']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // Ignora valori, notifica solo completamento
);

// Processo inizializzazione 2: Caricamento file config
const loadConfig$ = from(['Caricamento file config...', 'Parsing config...', 'Config applicata']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// Processo inizializzazione 3: Autenticazione utente
const authenticate$ = from(['Verifica credenziali...', 'Validazione token...', 'Autenticazione completata']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// Esegui tutti i processi di inizializzazione
addLog('Inizializzazione avviata...', 'orange');

forkJoin([
  initDatabase$,
  loadConfig$,
  authenticate$
]).subscribe({
  complete: () => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#e8f5e9';
    completeMessage.style.color = 'green';
    completeMessage.style.fontWeight = 'bold';
    completeMessage.textContent = '✅ Tutta l\'inizializzazione completata! L\'applicazione può avviarsi.';
    addLog('Avvio applicazione', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ Errore inizializzazione: ${err.message}`;
  }
});
```

- I log dettagliati per ogni processo di inizializzazione vengono mostrati, ma i valori sono ignorati.
- Un messaggio di completamento viene visualizzato quando tutti i processi completano.

## 🆚 Confronto con Operatori Simili

### ignoreElements vs filter(() => false) vs take(0)

| Operatore | Elaborazione Valori | Notifica Completamento | Caso d'Uso |
|:---|:---|:---|:---|
| `ignoreElements()` | Ignora tutti | Fa passare | **Serve solo completamento** (raccomandato) |
| `filter(() => false)` | Filtra tutti | Fa passare | Filtraggio condizionale (tutti esclusi per caso) |
| `take(0)` | Completa immediatamente | Fa passare | Vuoi completare immediatamente |

**Raccomandato**: Usa `ignoreElements()` quando intenzionalmente ignori tutti i valori. Rende chiaro l'intento del codice.

## 🔄 Gestione Notifiche di Errore

`ignoreElements` ignora i valori ma **fa passare le notifiche di errore**.

```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('Si è verificato un errore'))
).pipe(
  ignoreElements()
);

// Caso successo
success$.subscribe({
  next: v => console.log('Valore:', v),
  complete: () => console.log('✅ Completo'),
  error: err => console.error('❌ Errore:', err.message)
});
// Output: ✅ Completo

// Caso errore
error$.subscribe({
  next: v => console.log('Valore:', v),
  complete: () => console.log('✅ Completo'),
  error: err => console.error('❌ Errore:', err.message)
});
// Output: ❌ Errore: Si è verificato un errore
```

## ⚠️ Note

### 1. I Side Effect Vengono Eseguiti

`ignoreElements` ignora i valori ma i side effect (come `tap`) vengono eseguiti.

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs';

of(1, 2, 3).pipe(
  tap(v => console.log('Side effect:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('Valore:', v),
  complete: () => console.log('Completo')
});
// Output:
// Side effect: 1
// Side effect: 2
// Side effect: 3
// Completo
```

### 2. Uso con Observable Infiniti

Con Observable infiniti, la subscription continua per sempre poiché il completamento non arriva mai.

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs';

// ❌ Esempio sbagliato: Non completa
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Completo') // Non chiamato
});

// ✅ Esempio corretto: Completa con take
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('Completo') // Chiamato dopo 5 secondi
});
```

### 3. Tipo TypeScript

Il valore restituito da `ignoreElements` è di tipo `Observable<never>`.

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const numbers$: Observable<number> = of(1, 2, 3);

// Il risultato di ignoreElements è Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value è di tipo never, quindi questo blocco non viene eseguito
    console.log(value);
  },
  complete: () => console.log('Solo completamento')
});
```

## 📚 Operatori Correlati

- **[filter](/it/guide/operators/filtering/filter)** - Filtra valori in base a condizioni
- **[take](/it/guide/operators/filtering/take)** - Ottieni solo primi N valori
- **[skip](/it/guide/operators/filtering/skip)** - Salta primi N valori
- **[tap](https://rxjs.dev/api/operators/tap)** - Esegui side effect (documentazione ufficiale)

## Riepilogo

L'operatore `ignoreElements` ignora tutti i valori e fa passare solo completamento ed errore.

- ✅ Ideale quando serve solo la notifica di completamento
- ✅ I side effect (tap) vengono eseguiti
- ✅ Fa passare anche le notifiche di errore
- ✅ Intento più chiaro di `filter(() => false)`
- ⚠️ Non completa con Observable infiniti
- ⚠️ Il tipo del valore restituito è `Observable<never>`
- ⚠️ I valori sono completamente ignorati ma i side effect vengono eseguiti
