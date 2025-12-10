---
description: takeLast è un operatore di filtraggio RxJS che emette solo gli ultimi N valori quando lo stream Observable completa. È ideale per scenari dove servono solo gli ultimi valori dell'intero stream, come ottenere le ultime voci di log, visualizzare i top N elementi in una classifica e riepiloghi dati finali al completamento. Non può essere usato con stream infiniti perché mantiene i valori in un buffer fino al completamento.
---

# takeLast - Ottieni gli Ultimi N Valori

L'operatore `takeLast` emette solo gli ultimi N valori quando lo stream **completa**. Mantiene i valori in un buffer fino al completamento dello stream, poi li emette tutti insieme.


## 🔰 Sintassi e Utilizzo Base

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // Da 0 a 9

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9
```

**Flusso di operazione**:
1. Lo stream emette 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
2. Internamente mantiene gli ultimi 3 valori nel buffer
3. Lo stream completa
4. Emette i valori del buffer 7, 8, 9 in ordine

[🌐 Documentazione Ufficiale RxJS - `takeLast`](https://rxjs.dev/api/operators/takeLast)


## 🆚 Confronto con take

`take` e `takeLast` hanno comportamento opposto.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // Da 0 a 9

// take: Ottieni primi N valori
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Output: 0, 1, 2 (emessi immediatamente)

// takeLast: Ottieni ultimi N valori
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9 (emessi dopo attesa completamento)
```

| Operatore | Posizione | Timing Output | Comportamento Prima del Completamento |
|---|---|---|---|
| `take(n)` | Primi n valori | Output immediato | Auto-completa dopo n valori |
| `takeLast(n)` | Ultimi n valori | Output tutti insieme dopo completamento | Mantieni nel buffer |


## 💡 Pattern di Utilizzo Tipici

1. **Ottieni Ultime N Voci di Log**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App avviata' },
     { timestamp: 2, level: 'info' as const, message: 'Utente loggato' },
     { timestamp: 3, level: 'warn' as const, message: 'Query lenta rilevata' },
     { timestamp: 4, level: 'error' as const, message: 'Connessione fallita' },
     { timestamp: 5, level: 'info' as const, message: 'Retry riuscito' },
   ] as LogEntry[]);

   // Ottieni ultime 3 voci di log
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // Output:
   // [warn] Query lenta rilevata
   // [error] Connessione fallita
   // [info] Retry riuscito
   ```

2. **Ottieni Top N in Classifica**
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
   ] as Score[]);

   // Ottieni top 3
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Output: Charlie: 200, Dave: 180, Eve: 220
   ```


## ⚠️ Note Importanti

> [!WARNING]
> `takeLast` **attende fino al completamento dello stream**, quindi non funziona con stream infiniti. Inoltre, se n in `takeLast(n)` è grande, consuma molta memoria.

### 1. Non Può Essere Usato con Stream Infiniti

`takeLast` attende il completamento dello stream, quindi non funziona con stream infiniti.

```ts
import { interval } from 'rxjs';
import { takeLast } from 'rxjs';

// ❌ Esempio sbagliato: Usa takeLast con stream infinito
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);
// Nessun output (perché lo stream non completa mai)
```

**Soluzione**: Rendilo uno stream finito combinandolo con `take`

```ts
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs';

// ✅ Esempio corretto: Usa takeLast dopo averlo reso stream finito
interval(1000).pipe(
  take(10),      // Completa con primi 10 valori
  takeLast(3)    // Ottieni ultimi 3 da loro
).subscribe(console.log);
// Output: 7, 8, 9
```

### 2. Attenzione all'Utilizzo della Memoria

`takeLast(n)` mantiene gli ultimi n valori in un buffer, quindi n grande consuma memoria.

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

// ⚠️ Attenzione: Mantieni grande quantità di dati nel buffer
range(0, 1000000).pipe(
  takeLast(100000) // Mantieni 100.000 elementi in memoria
).subscribe(console.log);
```


## 🎯 Differenza da last

```ts
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs';

const numbers$ = range(0, 10);

// last: Solo ultimo 1 valore
numbers$.pipe(
  last()
).subscribe(console.log);
// Output: 9

// takeLast(1): Ultimo 1 valore (emesso come singolo valore, non array)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);
// Output: 9

// takeLast(3): Ultimi 3 valori
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Output: 7, 8, 9
```

| Operatore | Conteggio | Specifica Condizione | Caso d'Uso |
|---|---|---|---|
| `last()` | 1 valore | Possibile | Ultimo 1 valore o ultimo 1 valore che soddisfa condizione |
| `takeLast(n)` | n valori | Non possibile | Semplicemente ottieni ultimi n valori |


## 🎓 Riepilogo

### Quando Usare takeLast
- ✅ Quando hai bisogno degli ultimi N dati dallo stream
- ✅ Quando vuoi ottenere le ultime N voci di log o transazioni
- ✅ Quando il completamento dello stream è garantito
- ✅ Quando vuoi visualizzare riepilogo dati o top N elementi

### Quando Usare take
- ✅ Quando hai bisogno dei primi N dati dallo stream
- ✅ Quando vuoi ottenere risultati immediatamente
- ✅ Quando vuoi ottenere una porzione da uno stream infinito

### Note
- ⚠️ Non può essere usato con stream infiniti (non completa)
- ⚠️ n grande in `takeLast(n)` consuma memoria
- ⚠️ L'output viene fatto tutto insieme dopo il completamento (non emesso immediatamente)
- ⚠️ Spesso serve combinare con `take(n)` per rendere lo stream finito


## 🚀 Prossimi Passi

- **[take](/it/guide/operators/filtering/take)** - Impara come ottenere i primi N valori
- **[last](/it/guide/operators/filtering/last)** - Impara come ottenere l'ultimo 1 valore
- **[skip](/it/guide/operators/filtering/skip)** - Impara come saltare i primi N valori
- **[filter](/it/guide/operators/filtering/filter)** - Impara come filtrare in base a condizioni
- **[Esempi Pratici Operatori di Filtraggio](/it/guide/operators/filtering/practical-use-cases)** - Impara casi d'uso reali
