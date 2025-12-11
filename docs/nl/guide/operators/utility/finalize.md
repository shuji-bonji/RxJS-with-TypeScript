---
description: finalize is een RxJS utility operator die een proces definieert dat wordt uitgevoerd wanneer een Observable is voltooid, een fout heeft, of wordt afgemeld. Het is ideaal voor situaties die opschoning aan het einde van de stream vereisen, zoals resource-vrijgave, einde van laadweergave, en opschoonoperaties. Het zorgt ervoor dat operaties net zo betrouwbaar worden uitgevoerd als try-finally en helpt geheugenlekken te voorkomen.
---

# finalize - Verwerking bij voltooiing

De `finalize` operator definieert een proces dat wordt aangeroepen wanneer **Observable is voltooid, een fout heeft, of wordt afgemeld**.
Dit is ideaal voor "moet uitvoeren" processen zoals opschoning en UI-laad vrijgave.

## 🔰 Basissyntax en werking

```ts
import { of } from 'rxjs';
import { finalize } from 'rxjs';

of('Voltooid')
  .pipe(finalize(() => console.log('Stream is beëindigd')))
  .subscribe(console.log);
// Uitvoer:
// Voltooid
// Stream is beëindigd
```

In dit voorbeeld wordt het proces in `finalize` uitgevoerd na het uitzenden van één waarde in `of()`.
**Het wordt betrouwbaar aangeroepen voor zowel `complete` als `error`**.

[🌐 RxJS Officiële Documentatie - finalize](https://rxjs.dev/api/index/function/finalize)

## 💡 Typisch gebruiksvoorbeeld

Het volgende is een voorbeeld van het schakelen van laadweergave voor en na streaming.

```ts
import { of } from 'rxjs';
import { tap, delay, finalize } from 'rxjs';

let isLoading = false;

of('Data')
  .pipe(
    tap(() => {
      isLoading = true;
      console.log('Laden gestart');
    }),
    delay(1000),
    finalize(() => {
      isLoading = false;
      console.log('Laden beëindigd');
    })
  )
  .subscribe((value) => console.log('Opgehaald:', value));
// Uitvoer:
// Laden gestart
// Opgehaald: Data
// Laden beëindigd
```

## 🧪 Praktisch codevoorbeeld (met UI)

```ts
import { interval } from 'rxjs';
import { take, finalize, tap } from 'rxjs';

// Uitvoerweergavegebied
const finalizeOutput = document.createElement('div');
finalizeOutput.innerHTML = '<h3>finalize voorbeeld:</h3>';
document.body.appendChild(finalizeOutput);

// Laadindicator
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Data laden...';
loadingIndicator.style.backgroundColor = '#e0f7fa';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.borderRadius = '5px';
finalizeOutput.appendChild(loadingIndicator);

// Voortgangsweergave
const progressContainer = document.createElement('div');
progressContainer.style.marginTop = '10px';
finalizeOutput.appendChild(progressContainer);

// Voltooiingsbericht element
const completionMessage = document.createElement('div');
completionMessage.style.marginTop = '10px';
completionMessage.style.fontWeight = 'bold';
finalizeOutput.appendChild(completionMessage);

// Data ophalen simulatie
interval(500)
  .pipe(
    take(5), // Haal 5 waarden op
    tap((val) => {
      const progressItem = document.createElement('div');
      progressItem.textContent = `Verwerken item ${val + 1}...`;
      progressContainer.appendChild(progressItem);
    }),
    finalize(() => {
      loadingIndicator.style.display = 'none';
      completionMessage.textContent = 'Verwerking voltooid!';
      completionMessage.style.color = 'green';
    })
  )
  .subscribe({
    complete: () => {
      const successMsg = document.createElement('div');
      successMsg.textContent = 'Alle data succesvol geladen.';
      completionMessage.appendChild(successMsg);
    },
  });
```

## ✅ Samenvatting

- `finalize` wordt **altijd uitgevoerd** ongeacht voltooiing, fout, of handmatige beëindiging
- Ideaal voor opschoning en laadbeëindigingsprocessen
- Kan worden gecombineerd met andere operators (`tap`, `delay`, etc.) voor **veilige asynchrone opschoning**
