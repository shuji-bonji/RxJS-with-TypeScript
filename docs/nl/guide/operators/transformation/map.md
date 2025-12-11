---
description: De map operator is een basale conversiemethode die een functie toepast op elke waarde in een Observable om een nieuwe waarde te genereren, en wordt vaak gebruikt voor formulierformattering en API-responsverwerking.
---

# map - Pas een conversiefunctie toe op elke waarde

De `map` operator past een gespecificeerde functie toe op **elke waarde** in de stream om een nieuwe waarde na conversie te produceren.
Vergelijkbaar met de `Array.prototype.map` methode van een array, maar het werkt **op asynchrone streams**.


## 🔰 Basissyntax en gebruik

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3).pipe(
  map(value => value * 10)
).subscribe(console.log);
// Output: 10, 20, 30
```

Past de functie `value => value * 10` toe op elke waarde om een nieuwe waarde te produceren.

[🌐 RxJS Officiële Documentatie - map](https://rxjs.dev/api/index/function/map)


## 💡 Typische gebruikspatronen
- API-responsconversie (extraheer alleen noodzakelijke eigenschappen)
- Formattering van formulierinvoerdata
- Verwerken van getallen en strings in streams
- Extraheer alleen noodzakelijke data uit UI-gebeurtenissen


## 🧠 Praktisch codevoorbeeld (met UI)

Dit is een voorbeeld van het realtime weergeven van een ingevoerde numerieke waarde vermenigvuldigd met 2.

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Maak invoerveld
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Voer een getal in';
document.body.appendChild(input);

// Maak uitvoerveld
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Invoergebeurtenisstroom
fromEvent(input, 'input').pipe(
  map(event => Number((event.target as HTMLInputElement).value)),
  map(value => value * 2)
).subscribe(result => {
  output.textContent = `Verdubbelde waarde: ${result}`;
});
```

- De invoerwaarde wordt realtime verdubbeld en uitgevoerd.
- Een eenvoudige dataconversieketen wordt gerealiseerd door continu map toe te passen.
