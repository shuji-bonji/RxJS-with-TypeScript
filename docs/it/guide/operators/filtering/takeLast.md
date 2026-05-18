---
description: "takeLast è un operatore di filtraggio di RxJS che restituisce solo gli ultimi N valori al termine di un flusso di Observable. È ideale per le situazioni in cui è richiesto solo l'ultimo valore dell'intero flusso, come ad esempio ottenere l'ultimo conteggio nel log, visualizzare i primi N valori nella classifica o il riepilogo finale dei dati al completamento. Non può essere utilizzato con flussi infiniti, poiché viene mantenuto in un buffer fino al completamento."
---

# takeLast - per ottenere gli ultimi N valori

L'operatore takeLast emette solo gli ultimi N valori nel momento in cui lo stream è **completato**. Mantiene i valori in un buffer fino al completamento dello stream e li invia insieme dopo il completamento.

## 🔰 Sintassi e utilizzo di base

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0da (a)9a

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Uscita: 7, 8, 9
```

**Flusso operativo**:.
1. il flusso emette 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
2. mantenere internamente gli ultimi 3 nel buffer
3. flusso completato 4. valori del buffer 7, 8, 9
4. uscita dei valori del buffer 7, 8, 9 in sequenza

[🌐 Documentazione ufficiale di RxJS - takeLast](https://rxjs.dev/api/operators/takeLast)

## 🆚 Contrasto con take.

`take` e `takeLast` hanno un comportamento contrastante.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0da (a)9a

// take: Il primoNOttiene il primo
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Uscita: 0, 1, 2(uscita immediata)

// takeLast: Ottenere l'ultimoNOttiene il primo
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Uscita: 7, 8, 9(aspetta il completamento prima di emettere)
```

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0da (a)9a

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Uscita: 7, 8, 9

## 💡 Modello tipico di utilizzo

1. **Raccogliere le ultime N voci di registro**.

```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'warn' as const, message: 'Slow query detected' },
     { timestamp: 4, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 5, level: 'info' as const, message: 'Retry successful' },
   ] as LogEntry[]);

   // Ottieni il più recente3Recupera i log del
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // Uscita:
   // [warn] Slow query detected
   // [error] Connection failed
   // [info] Retry successful
   ```

2. **In cima alla classificaNRecuperare la parte superiore**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface Score {
     player: string;
     score: number;
   }

   const scores$ = from([
     { player: 'Alice', score: 100 },
     { player: 'Bob', score: 150 },
     { player: 'Charlie', score: 200 },
     { player: 'Dave', score: 180 },
     { player: 'Eve', score: 220 }
   ] as Score[]).pipe(
     // Assumere l'ordinamento per punteggio
   );

   // Ottenere la top3Recupera il
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Uscita: Charlie: 200, Dave: 180, Eve: 220
   ```

3. **Riepilogo finale al termine dell'elaborazione dei datiNRiepilogo dei casi**
   ```ts
   import { interval } from 'rxjs';
   import { take, map, takeLast } from 'rxjs';

   // Simulazione dei dati del sensore
   const sensorData$ = interval(100).pipe(
     take(20),
     map(i => ({
       id: i,
       temperature: 20 + Math.random() * 10
     }))
   );

   // Ottenere l'ultimo5Calcolo della temperatura media del caso
   sensorData$.pipe(
     takeLast(5)
   ).subscribe({
     next: data => {
       console.log(`dati${data.id}: ${data.temperature.toFixed(1)}°C`);
     },
     complete: () => {
       console.log('Ultimo5Acquisizione dei dati del caso completata');
     }
   });
   ```

## 🧠 Esempio pratico di codice (cronologia degli ingressi)

Esempio di visualizzazione degli ultimi valori3Questo è un esempio di visualizzazione degli ultimi valori inseriti dall'utente.

```

ts.
import { fromEvent, Subject } from 'rxjs';
import { takeLast } from 'rxjs';

// Creare gli elementi dell'interfaccia utente
const container = document.createElement('div');
document.body.appendChild(container);

const input = document.createElement('input');
input.placeholder = 'Inserisci un valore e dai Invio';
container.appendChild(input);

const submitButton = document.createElement('button');
submitButton.textContent = 'Mostra la cronologia (ultimi 3)';
container.appendChild(submitButton);

const historyDisplay = document.createElement('div');
historyDisplay.style.marginTop = '10px';
container.appendChild(historyDisplay);

// Soggetto per contenere i valori di input
const inputs$ = new Subject();.

// **IMPORTANTE**: impostare l'abbonamento takeLast per primo
inputs$.pipe(
  takeLast(3)
).subscribe({
  next: (valore) => {
    const item = document.createElement('div');
    item.textContent = `- ${value}`;
    historyDisplay.appendChild(item);
  },.
  complete: () => {
    const note = document.createElement('div');
    note.style.marginTop = '5px';
    note.style.color = 'grey';
    note.textContent = '(Ricarica la pagina per digitare di nuovo)';
    historyDisplay.appendChild(nota);

    // Disabilitare i campi di input e i pulsanti
    input.disabled = true;
    submitButton.disabled = true;
  }
});

// Aggiungere l'input con il tasto Invio
fromEvent<KeyboardEvent>(input, 'keydown').subscribe(event => {
  if (event.key === 'Enter' && input.value.trim()) {
    inputs$.next(input.value);
    console.log(`Aggiungi: ${input.value}`);
    input.value = '';
  }
});

// Completare con il clic sul pulsante e visualizzare la cronologia
fromEvent(submitButton, 'click').subscribe(() => {
  historyDisplay.innerHTML = '<strong>Storia (ultime 3):</strong><br>';
  inputs$.complete(); // completa il flusso → takeLast spara
});

```

> [!IMPORTANT]
> **Punti chiave**:
> - `takeLast(3)` Sottoscrivere il**Prima di tutto.**deve essere impostato prima
> - Quando si fa clic sul pulsante, viene emesso l'ultimo valore ricevuto fino a quel punto `complete()` viene emesso l'ultimo dei valori ricevuti fino a quel momento.3L'ultimo dei valori ricevuti fino a quel momento viene emesso.
> - `complete()` Dopo la chiamata**Dopo aver chiamato**a `subscribe` nessun valore fluirà se si chiama

## ⚠️ Un punto importante da notare

> [!WARNING]
> `takeLast` è di attendere che il flusso**Attendere fino al completamento**Pertanto, non funziona con flussi infiniti. Inoltre, l'opzione`takeLast(n)` del metodonè grande e consuma molta memoria.

### 1. Non può essere utilizzato con flussi infiniti.

`takeLast` non funziona con flussi infiniti perché attende che il flusso sia completato.

```

ts.
import { interval } from 'rxjs';
import { takeLast } from 'rxjs';

// ❌ Cattivo esempio: utilizzo di takeLast con flussi infiniti
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);.
// Non viene emesso nulla (perché il flusso non viene mai completato)

```

**Soluzione.**: `take` Utilizzare un flusso finito in combinazione con

```

ts.
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs';

// ✅ Buon esempio: flusso finito e poi uso di takeLast
interval(1000).pipe(
  take(10), // Completa con i primi 10
  takeLast(3) // prende gli ultimi 3
).subscribe(console.log);.
// Output: 7, 8, 9

```

### 2. Prestare attenzione all'uso della memoria

`takeLast(n)` non funziona con i flussi finiti, perché trattiene l'ultimo pezzo danpezzo da tenere nel buffer,nè grande, consuma più memoria.

```

ts.
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

// ⚠️ Nota: grandi quantità di dati vengono conservate in un buffer
range(0, 1000000).pipe(
  takeLast(100000) // 100.000 record tenuti in memoria
).subscribe(console.log);.

```

## 🎯 last La differenza tra

```

ts.
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs';

const numbers$ = range(0, 10);

// last: solo l'ultimo
numbers$.pipe(
  last()
).subscribe(console.log);
// uscita: 9

// takeLast(1): ultimo (output come valore singolo, non come array)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);.
// Uscita: 9

// takeLast(3): ultimo 3
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Uscita: 7, 8, 9

```

| operatore | Numero di acquisizioni | Specifica della condizione | Caso d'uso |
|---|---|---|---|
| `last()` | 1Numero di | Possibile | Ottenere l'ultimo1Pezzi o l'ultimo pezzo che soddisfa la condizione1Numero di |
| `takeLast(n)` | nNumero di | Impossibile | Ottenere l'ultimonOttenere semplicemente l'ultimo pezzo che soddisfa la condizione |

## 📋 Utilizzo sicuro per i tipi

TypeScript Questo è un esempio di implementazione type-safe che fa uso dei generici in

```

ts.
import { Observable, from } from 'rxjs';
import { takeLast } from 'rxjs';

interfaccia Transazione {
  id: stringa;
  importo: numero
  timestamp: data;
  status: 'pending' | 'completed' | 'failed'; }
}

function getRecentTransactions(
  transactions$: Observable,.
  count: numero
): Observable {
  return transactions$.pipe(
    takeLast(count)
  );
}

// Esempio di utilizzo
const transazioni$ = from([.
  { id: '1', importo: 100, timestamp: new Date('2025-01-01'), status: 'completato' as const}
  { id: '2', importo: 200, timestamp: new Date('2025-01-02'), status: 'completed' as const }.
  { id: '3', importo: 150, timestamp: new Date('2025-01-03'), status: 'pending' as const }
  { id: '4', importo: 300, timestamp: new Date('2025-01-04'), status: 'completed' as const }
  { id: '5', importo: 250, timestamp: new Date('2025-01-05'), status: 'failed' as const }
] come Transaction[]);.

// Ottenere le tre transazioni più recenti
getRecentTransactions(transactions$, 3).subscribe(tx => {
  console.log(`${tx.id}: ${tx.amount} yen (${tx.status})`);
});
// Output:.
// 3: 150 yen (in attesa)
// 4: 300 yen (completato)
// 5: 250 yen (fallito)

```

## 🔄 skip e takeLast della combinazione di

La parte centrale del valore viene esclusa e viene recuperata solo l'ultima parte del valore.NSolo l'ultimo può essere recuperato.

```

ts
import { range } from 'rxjs';
import { skip, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // da 0 a 9

// salta i primi 5 e prende i restanti 3
numbers$.pipe(
  skip(5), // salta 0, 1, 2, 3, 4
  takeLast(3) // prende gli ultimi 3 dai rimanenti 5, 6, 7, 8, 9
).subscribe(console.log);.
// Uscita: 7, 8, 9
```

## 🎓 Sommario

### Quando si dovrebbe usare takeLast.
- ✅ Se si ha bisogno degli ultimi N dati in un flusso
- ✅ Se si vogliono ottenere gli ultimi N log o transazioni
- ✅ Se il completamento del flusso è garantito
- ✅ Se si desidera visualizzare un riepilogo o i primi N record di dati

### Quando si dovrebbe usare take.
- ✅ Se si vogliono i primi N dati del flusso
- ✅ Se si vogliono ottenere i risultati immediatamente
- ✅ Se si vuole ottenere una parte di un flusso infinito

### Note.
- ⚠️ Non può essere usato con flussi infiniti (perché non si completano).
- ⚠️ Un grande n in takeLast(n)` consuma memoria
- ⚠️ L'output viene compilato dopo il completamento (non immediatamente)
- ⚠️ Spesso deve essere combinato con `take(n)` per ottenere un flusso finito.

## 🚀 Passo successivo.

- **[take](. /take)** - imparare come ottenere i primi n valori.
- **[last](. /last)** - impara a ottenere l'ultimo 1 valore.
- **[skip](. /skip)** - impara a saltare i primi N valori.
- **[filter](. /filter)** - per imparare a filtrare in base a delle condizioni
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - per imparare a utilizzare casi d'uso reali.
