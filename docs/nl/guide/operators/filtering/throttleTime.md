---
description: De throttleTime operator dunt hoogfrequente gebeurtenissen efficiënt uit door alleen de eerste waarde binnen een gespecificeerd tijdsinterval door te laten en volgende waarden te negeren. Het is ideaal voor realtime gebeurtenisoptimalisatie zoals scrollen of muisbewegingen.
titleTemplate: ':title'
---

# throttleTime - Laat eerste waarde door en negeer nieuwe waarden gedurende gespecificeerde tijd

De `throttleTime` operator laat de eerste uitgegeven waarde door en negeert volgende waarden die binnen een gespecificeerd tijdsinterval worden uitgegeven.
Het geeft niet de laatste waarde op regelmatige intervallen uit, maar **laat alleen de eerste waarde die het ontvangt door en negeert volgende waarden gedurende die periode**.

Dit is nuttig voor het uitdunnen van streams die frequent vuren, zoals scroll-gebeurtenissen en muisbewegingsgebeurtenissen.


## 🔰 Basissyntax en gebruik

```ts
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

fromEvent(document, 'click')
  .pipe(throttleTime(2000))
  .subscribe(() => console.log('Geklikt!'));

```

- Ontvangt alleen de eerste klikgebeurtenis elke 2 seconden en negeert volgende klikken.

[🌐 RxJS Officiële Documentatie - `throttleTime`](https://rxjs.dev/api/operators/throttleTime)


## 💡 Typische gebruikspatronen

- Optimalisatie van gebeurtenisafhandeling voor scrollen en muisbewegingen
- Preventie van meerdere inzendingen door opeenvolgende knopdrukken
- Uitdunning van realtime datastreams


## 🧠 Praktisch codevoorbeeld (met UI)

Wanneer de muis wordt bewogen, wordt positie-informatie elke 100 milliseconden weergegeven.

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

// Maak uitvoergebied
const container = document.createElement('div');
container.style.height = '200px';
container.style.border = '1px solid #ccc';
container.style.padding = '10px';
container.textContent = 'Beweeg uw muis binnen dit gebied';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
document.body.appendChild(positionDisplay);

// Muisbewegingsgebeurtenis
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => ({
    x: event.clientX,
    y: event.clientY
  })),
  throttleTime(100)
).subscribe(position => {
  positionDisplay.textContent = `Muispositie: X=${position.x}, Y=${position.y}`;
});
```

- Beperkt frequent vurende muisbewegingsgebeurtenissen tot elke 100ms en toont alleen de meest recente positie.
