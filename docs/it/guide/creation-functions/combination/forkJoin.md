---
description: La Funzione di Creazione forkJoin emette l'ultimo valore di ogni Observable come array o oggetto dopo che tutti gli Observable multipli sono completati. Questo è ideale quando si eseguono richieste API multiple in parallelo e tutti i risultati sono disponibili prima dell'elaborazione.
---

# forkJoin - emetti tutti gli ultimi valori insieme

`forkJoin` è una Funzione di Creazione che emette l'ultimo valore di ogni Observable come array o oggetto dopo che **tutti** gli Observable sono stati completati.
Questo è molto utile quando vuoi usare tutti gli Observable in una volta.


## Sintassi base e utilizzo

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

const user$ = of('Utente A').pipe(delay(1000));
const posts$ = of('Lista post').pipe(delay(1500));

forkJoin([user$, posts$]).subscribe(([user, posts]) => {
  console.log(user, posts);
});

// Output:
// Utente A Lista post
```

- Aspetta finché tutti gli Observable non sono `complete`.
- Solo l'**ultimo valore emesso** di ogni Observable viene compilato ed emesso.

[🌐 Documentazione Ufficiale RxJS - `forkJoin`](https://rxjs.dev/api/index/function/forkJoin)


## Pattern di utilizzo tipici

- **Esegui richieste API multiple in parallelo e riepiloga tutti i risultati**
- **Ottieni più dataset necessari per il caricamento iniziale in una volta**
- **Ottieni tutti i dati correlati in una volta e disegna il rendering dello schermo in una volta**


## Esempi di codice pratici (con UI)

Simula richieste API multiple e visualizzale insieme quando tutti i risultati sono disponibili.

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.innerHTML = '<h3>Esempio pratico forkJoin:</h3>';
document.body.appendChild(output);

// Stream dati fittizi
const user$ = of({ id: 1, name: 'Mario Rossi' }).pipe(delay(2000));
const posts$ = of([{ id: 1, title: 'Post 1' }, { id: 2, title: 'Post 2' }]).pipe(delay(1500));
const weather$ = of({ temp: 22, condition: 'Soleggiato' }).pipe(delay(1000));

// Messaggio di caricamento
const loading = document.createElement('div');
loading.textContent = 'Caricamento dati...';
loading.style.color = 'blue';
output.appendChild(loading);

// Emetti tutto insieme dopo che tutte le richieste sono completate
forkJoin({
  user: user$,
  posts: posts$,
  weather: weather$
}).subscribe(result => {
  output.removeChild(loading);

  const pre = document.createElement('pre');
  pre.textContent = JSON.stringify(result, null, 2);
  pre.style.background = '#f5f5f5';
  pre.style.padding = '10px';
  pre.style.borderRadius = '5px';
  output.appendChild(pre);

  const summary = document.createElement('div');
  summary.textContent = `Utente: ${result.user.name}, Meteo: ${result.weather.condition}, Post: ${result.posts.length}`;
  output.appendChild(summary);
});
```

- Prima viene visualizzato il caricamento,
- Quando tutti i dati sono disponibili, i risultati vengono disegnati insieme.
