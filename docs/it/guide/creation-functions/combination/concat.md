---
description: "Spiegazione di come combinare più Observable in ordine con la Creation Function concat. Poiché il successivo inizia dopo il completamento del primo Observable, può essere utilizzata per esecuzione passo-passo, visualizzazione UI sequenziale e chiamate API consecutive. Introduzione all'inferenza dei tipi TypeScript ed esempi pratici."
---

# concat - Combina stream in ordine

`concat` è una Creation Function che **esegue sequenzialmente** più Observable nell'ordine specificato.
L'emissione del successivo Observable inizia dopo il `complete` del precedente.

## Sintassi base e uso

```ts
import { concat, of, delay } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));

concat(obs1$, obs2$).subscribe(console.log);
// Output: A → B → C → D
```

- L'emissione di `obs2$` inizia dopo il completamento di tutte le emissioni di `obs1$`.
- Il punto chiave è che gli stream fluiscono "in ordine" e non simultaneamente.

[🌐 RxJS Official Documentation - `concat`](https://rxjs.dev/api/index/function/concat)


## Pattern di utilizzo tipici

- **Elaborazione passo-passo**: Quando si desidera procedere al passo successivo dopo il completamento del processo precedente
- **Richieste API con ordine garantito**: Quando si desidera eseguire una serie di operazioni asincrone in ordine
- **Controllo di eventi UI come animazioni o notifiche in cui l'ordine è importante**

## Esempio di codice pratico (con UI)

Esempio di **visualizzazione in ordine** di messaggio di caricamento ed elenco dati.

```ts
import { concat, of, timer } from 'rxjs';
import { map, take } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.innerHTML = '<h3>Esempio pratico di concat:</h3>';
document.body.appendChild(output);

// Stream di caricamento
const loading$ = timer(0, 1000).pipe(
  map((count) => `⏳ Caricamento in corso... (${count + 1}s)`),
  take(3) // Emette solo per 3 secondi
);

// Stream elenco dati
const data$ = of('🍎 Apple', '🍌 Banana', '🍇 Grape');

// Visualizza in ordine con concat
concat(loading$, data$).subscribe((value) => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});
```

- Prima viene visualizzato il messaggio di caricamento 3 volte,
- Poi l'elenco dati viene visualizzato in ordine.
- Utilizzando **concat**, è facile realizzare una "visualizzazione graduale" naturale.


## Operatori correlati

- **[concatWith](/it/guide/operators/combination/concatWith)** - Versione Pipeable Operator (uso all'interno della pipeline)
- **[concatMap](/it/guide/operators/transformation/concatMap)** - Mappa e combina ciascun valore sequenzialmente
- **[merge](/it/guide/creation-functions/combination/merge)** - Creation Function per combinazione parallela
