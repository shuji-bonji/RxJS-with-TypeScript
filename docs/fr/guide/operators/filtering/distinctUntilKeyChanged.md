---
description: "L'opérateur distinctUntilKeyChanged se concentre sur une propriété spécifique dans un flux d'objets et n'émet que lorsque sa valeur est différente de la précédente. Ignore efficacement les données en double consécutives, utile pour la détection de changement d'état et l'optimisation des mises à jour de liste."
---

# distinctUntilKeyChanged - Détecter uniquement les changements d'une propriété spécifique

L'opérateur `distinctUntilKeyChanged` se concentre sur une clé (propriété) spécifique d'un objet et n'émet que lorsque sa valeur est différente de la précédente.
Pratique pour ignorer efficacement les doublons consécutifs.


## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { distinctUntilKeyChanged } from 'rxjs';

const users = [
  { id: 1, name: 'Tanaka' },
  { id: 2, name: 'Tanaka' }, // Même name donc ignoré
  { id: 3, name: 'Sato' },
  { id: 4, name: 'Suzuki' },
  { id: 5, name: 'Suzuki' }, // Même name donc ignoré
  { id: 6, name: 'Tanaka' }
];

from(users).pipe(
  distinctUntilKeyChanged('name')
).subscribe(console.log);

// Sortie:
// { id: 1, name: 'Tanaka' }
// { id: 3, name: 'Sato' }
// { id: 4, name: 'Suzuki' }
// { id: 6, name: 'Tanaka' }
```

- N'émet que lorsque la valeur de la propriété `name` spécifiée change.
- Les autres propriétés (comme `id`) ne sont pas prises en compte pour la comparaison.

[🌐 Documentation officielle RxJS - `distinctUntilKeyChanged`](https://rxjs.dev/api/operators/distinctUntilKeyChanged)


## 💡 Patterns d'utilisation typiques

- Mettre à jour l'affichage de liste uniquement quand une propriété spécifique change
- Détecter uniquement les changements d'attributs spécifiques dans les flux d'événements
- Contrôler la suppression des doublons par clé


## 🧠 Exemple de code pratique (avec UI)

Entrez un nom dans le champ de texte et appuyez sur Entrée pour l'enregistrer.
**Si le même nom est saisi consécutivement, il est ignoré**, et seul un nom différent est ajouté à la liste.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, scan, distinctUntilKeyChanged } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
document.body.appendChild(output);

const title = document.createElement('h3');
title.textContent = 'Exemple pratique de distinctUntilKeyChanged';
output.appendChild(title);

// Formulaire de saisie
const input = document.createElement('input');
input.placeholder = 'Entrer un nom et appuyer sur Entrée';
document.body.appendChild(input);

// Flux d'événements de saisie
fromEvent<KeyboardEvent>(input, 'keydown').pipe(
  filter((e) => e.key === 'Enter'),
  map(() => input.value.trim()),
  filter((name) => name.length > 0),
  scan((_, name, index) => ({ id: index + 1, name }), { id: 0, name: '' }),
  distinctUntilKeyChanged('name')
).subscribe((user) => {
  const item = document.createElement('div');
  item.textContent = `Entrée utilisateur: ID=${user.id}, Nom=${user.name}`;
  output.appendChild(item);
});
```

- Si le même nom est saisi consécutivement, il est ignoré.
- Affiché uniquement quand un nouveau nom est saisi.
