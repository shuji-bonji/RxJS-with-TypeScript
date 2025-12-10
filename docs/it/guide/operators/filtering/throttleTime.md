---
description: L'operatore throttleTime riduce efficientemente gli eventi ad alta frequenza permettendo solo al primo valore di passare entro un intervallo di tempo specificato e ignorando i valori successivi. È ideale per l'ottimizzazione di eventi real-time come scrolling o movimento del mouse.
titleTemplate: ':title'
---

# throttleTime - Passa il Primo Valore e Ignora i Nuovi per il Tempo Specificato

L'operatore `throttleTime` passa il primo valore emesso e ignora i valori successivi emessi entro un intervallo di tempo specificato.
Non emette l'ultimo valore a intervalli regolari, ma piuttosto **passa solo il primo valore che riceve e ignora i valori successivi durante quel periodo**.

Questo è utile per ridurre gli stream che si attivano frequentemente, come eventi di scroll e movimento del mouse.


## 🔰 Sintassi e Utilizzo Base

```ts
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

fromEvent(document, 'click')
  .pipe(throttleTime(2000))
  .subscribe(() => console.log('Cliccato!'));

```

- Riceve solo il primo evento click ogni 2 secondi e ignora i click successivi.

[🌐 Documentazione Ufficiale RxJS - `throttleTime`](https://rxjs.dev/api/operators/throttleTime)


## 💡 Pattern di Utilizzo Tipici

- Ottimizzazione della gestione eventi per scrolling e movimento del mouse
- Prevenzione di invii multipli dovuti a pressioni consecutive del bottone
- Riduzione dello stream di dati real-time


## 🧠 Esempio di Codice Pratico (con UI)

Quando il mouse viene mosso, le informazioni sulla posizione vengono visualizzate ogni 100 millisecondi.

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

// Crea area di output
const container = document.createElement('div');
container.style.height = '200px';
container.style.border = '1px solid #ccc';
container.style.padding = '10px';
container.textContent = 'Muovi il mouse in quest\'area';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
document.body.appendChild(positionDisplay);

// Evento movimento mouse
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => ({
    x: event.clientX,
    y: event.clientY
  })),
  throttleTime(100)
).subscribe(position => {
  positionDisplay.textContent = `Posizione mouse: X=${position.x}, Y=${position.y}`;
});
```

- Limita gli eventi di movimento del mouse che si attivano frequentemente a ogni 100ms e visualizza solo la posizione più recente.
