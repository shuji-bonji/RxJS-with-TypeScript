---
description: "L'opérateur last récupère uniquement la dernière valeur lorsque le flux se termine, ou la dernière valeur qui correspond à une condition. Explique la différence avec first(), comment définir les valeurs par défaut, la gestion de EmptyError et l'implémentation TypeScript type-safe avec des exemples de code pratiques. Présente également la différence avec takeLast()."
---

# last - Obtenir Dernière Valeur

L'opérateur `last` récupère uniquement **la dernière valeur**, ou **la dernière valeur satisfaisant une condition** d'un flux, puis termine le flux.


## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { last } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Récupérer uniquement la dernière valeur
numbers$.pipe(
  last()
).subscribe(console.log);

// Récupérer uniquement la dernière valeur satisfaisant la condition
numbers$.pipe(
  last(n => n < 5)
).subscribe(console.log);

// Sortie:
// 5
// 4
```

- `last()` émet **la dernière valeur émise** à la fin du flux.
- Avec une condition, seule **la dernière valeur satisfaisant la condition** est récupérée.
- Si aucune valeur ne satisfait la condition, une erreur est émise.

[🌐 Documentation officielle RxJS - `last`](https://rxjs.dev/api/operators/last)


## 💡 Patterns d'utilisation typiques

- Récupérer le dernier élément de données filtrées
- Récupérer le dernier état à la fin du flux
- Extraire la dernière opération importante d'une session ou d'un journal d'opérations


## 🧠 Exemple de code pratique (avec UI)

Récupère et affiche la dernière valeur inférieure à 5 parmi les 5 nombres saisis.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, take, last } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de last :</h3>';
document.body.appendChild(output);

// Création du champ de saisie
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Entrez un nombre et appuyez sur Entrée';
document.body.appendChild(input);

// Flux d'événements de saisie
fromEvent<KeyboardEvent>(input, 'keydown')
  .pipe(
    filter((e) => e.key === 'Enter'),
    map(() => parseInt(input.value, 10)),
    take(5), // Prendre les 5 premiers puis terminer
    filter((n) => !isNaN(n) && n < 5), // Seuls les nombres inférieurs à 5 passent
    last() // Récupérer la dernière valeur inférieure à 5
  )
  .subscribe({
    next: (value) => {
      const item = document.createElement('div');
      item.textContent = `Dernière valeur inférieure à 5 : ${value}`;
      output.appendChild(item);
    },
    complete: () => {
      const complete = document.createElement('div');
      complete.textContent = 'Terminé';
      complete.style.fontWeight = 'bold';
      output.appendChild(complete);
    },
  });

```
1. Entrez 5 nombres et appuyez sur Entrée
2. Parmi les nombres saisis, seuls ceux « inférieurs à 5 » sont sélectionnés
3. Seule la dernière valeur inférieure à 5 saisie est affichée
4. Le flux se termine naturellement
