---
description: "switchMap annule l'Observable précédent et passe au plus récent. Idéal pour la recherche en direct, le changement de navigation, l'enregistrement automatique et d'autres cas d'utilisation, réalisant un traitement asynchrone sûr avec l'inférence de type TypeScript. Explique également les différences avec mergeMap et concatMap."
---

# switchMap - Annuler l'Observable précédent et passer au plus récent

L'opérateur `switchMap` génère un nouvel Observable pour chaque valeur du flux d'entrée, **annulant l'Observable précédent et basculant uniquement sur l'Observable le plus récent**.
C'est parfait pour des cas comme les formulaires de recherche où l'on veut que seule la valeur la plus récente soit valide.

## 🔰 Syntaxe de base et utilisation

```ts
import { of } from 'rxjs';
import { delay, switchMap } from 'rxjs';

of('A', 'B', 'C').pipe(
  switchMap(value =>
    of(`${value} terminé`).pipe(delay(1000))
  )
).subscribe(console.log);

// Sortie :
// C terminé
```

- Un nouvel Observable est créé pour chaque valeur.
- Cependant, **l'Observable précédent est annulé dès qu'une nouvelle valeur arrive**.
- En fin de compte, seul `C` est produit.

[🌐 Documentation officielle RxJS - switchMap](https://rxjs.dev/api/operators/switchMap)

## 💡 Modes d'utilisation typiques

- Formulaire de saisie avec autocomplétion
- Fonctionnalité de recherche en direct (seule la dernière entrée est valide)
- Chargement de ressources lors des changements de navigation ou de routage
- Lorsque vous souhaitez basculer les actions de l'utilisateur vers la plus récente

## 🧠 Exemple de code pratique (avec interface utilisateur)

Lorsque des caractères sont saisis dans la boîte de recherche, une requête API est immédiatement envoyée, affichant les résultats **uniquement pour la dernière entrée**.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

// Création du champ de saisie
const searchInput = document.createElement('input');
searchInput.placeholder = 'Rechercher par nom d\'utilisateur';
document.body.appendChild(searchInput);

// Zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Traitement des événements de saisie
fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  map(event => (event.target as HTMLInputElement).value.trim()),
  switchMap(term => {
    if (term === '') {
      return of([]);
    }
    return ajax.getJSON(`https://jsonplaceholder.typicode.com/users?username_like=${term}`);
  })
).subscribe(users => {
  output.innerHTML = '';

  (users as any[]).forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.username;
    output.appendChild(div);
  });
});
```

- Chaque changement de saisie annule la requête précédente.
- Seuls les utilisateurs correspondant au dernier terme de recherche sont affichés.
