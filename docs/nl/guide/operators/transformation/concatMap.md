---
description: ConcatMap is een conversieoperator die elke Observable op zijn beurt verwerkt en wacht op de volgende tot de vorige is voltooid, ideaal voor scenario's waar uitvoeringsvolgorde belangrijk is, zoals seriële uitvoering van API-aanroepen of garantie van bestandsuploadvolgordes. TypeScript type-inferentie maakt type-veilige asynchrone ketening mogelijk, en de verschillen met mergeMap en switchMap worden ook uitgelegd.
---

# concatMap - Voer elke Observable op volgorde uit

De `concatMap` operator converteert elke waarde in de invoerstroom naar een Observable en **voert ze op volgorde uit en koppelt ze aaneen**.
Het start de volgende Observable **niet tot de vorige Observable is voltooid**.

## 🔰 Basissyntax en gebruik

```ts
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  concatMap(value =>
    of(`${value} voltooid`).pipe(delay(1000))
  )
).subscribe(console.log);

// Output (op volgorde):
// A voltooid
// B voltooid
// C voltooid
```
- Converteert elke waarde naar een Observable.
- De volgende Observable wordt uitgevoerd nadat de vorige Observable is voltooid.

[🌐 RxJS Officiële Documentatie - concatMap](https://rxjs.dev/api/index/function/concatMap)

## 💡 Typische gebruikspatronen
- Uitvoering van volgorde-kritieke API-verzoeken
- Queue-gebaseerde taakverwerking
- Controle van animaties en stapsgewijze UI
- Berichten verzendproces waar verzendvolgorde belangrijk is


## 🧠 Praktisch codevoorbeeld (met UI)

Dit is een voorbeeld waar een verzoek wordt gegenereerd elke keer dat een knop wordt geklikt en de verzoeken worden altijd op volgorde verwerkt.

```ts
import { fromEvent, of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

// Maak knop
const button = document.createElement('button');
button.textContent = 'Verstuur verzoek';
document.body.appendChild(button);

// Uitvoergebied
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Klikgebeurtenis
fromEvent(button, 'click')
  .pipe(
    concatMap((_, index) => {
      const requestId = index + 1;
      console.log(`Verzoek ${requestId} gestart`);
      return of(`Respons ${requestId}`).pipe(delay(2000));
    })
  )
  .subscribe((response) => {
    const div = document.createElement('div');
    div.textContent = `✅ ${response}`;
    output.appendChild(div);
  });

```

- Elk verzoek wordt altijd op volgorde verzonden en voltooid.
- Het volgende verzoek wordt verstuurd nadat het vorige verzoek is voltooid.
