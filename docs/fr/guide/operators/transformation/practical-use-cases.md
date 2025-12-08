---
description: "Explique les cas d'utilisation pratiques des opérateurs de transformation RxJS (map, mergeMap, switchMap, concatMap, scan, buffer, etc.). Présente des patterns pratiques de traitement et transformation flexibles des données avec des exemples de code TypeScript, incluant le traitement des entrées utilisateur, le formatage des réponses API, les requêtes imbriquées, l'agrégation de données, et le traitement par lots. Apprenez des patterns de transformation fréquemment utilisés en pratique."
---

# Patterns de transformation pratiques

Les opérateurs de transformation sont l'un des groupes d'opérateurs les plus fréquemment utilisés dans RxJS.
Ils jouent un rôle essentiel dans la programmation réactive pour un traitement et une transformation flexibles des données.

Dans cette section, nous organisons les patterns d'utilisation des opérateurs de transformation en présentant des exemples pratiques typiques.


## 💬 Patterns d'utilisation typiques

| Pattern | Opérateurs représentatifs | Description |
|:---|:---|:---|
| Transformation simple de valeurs | `map` | Applique une fonction de transformation à chaque valeur |
| Traitement d'accumulation/agrégation | `scan`, `reduce` | Accumule les valeurs séquentiellement |
| Traitement asynchrone imbriqué | `mergeMap`, `switchMap`, `concatMap`, `exhaustMap` | Génère et combine des Observables |
| Traitement par lots/regroupement | `bufferTime`, `bufferCount`, `windowTime` | Traite ensemble/gère la division |
| Extraction de propriétés | `pluck` | Extrait un champ spécifique d'un objet |


## Validation et transformation des entrées utilisateur

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

// Champ de saisie
const emailInput = document.createElement('input');
const emailStatus = document.createElement('p');
document.body.appendChild(emailInput);
document.body.appendChild(emailStatus);

// Fonction de validation d'email
function isValidEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

// Traitement de l'entrée
fromEvent(emailInput, 'input')
  .pipe(
    debounceTime(400),
    map((event) => (event.target as HTMLInputElement).value.trim()),
    distinctUntilChanged(),
    map((email) => {
      if (!email) {
        return {
          isValid: false,
          message: 'Veuillez entrer une adresse email',
          value: email,
        };
      }

      if (!isValidEmail(email)) {
        return {
          isValid: false,
          message: 'Veuillez entrer une adresse email valide',
          value: email,
        };
      }

      return {
        isValid: true,
        message: 'L\'adresse email est valide',
        value: email,
      };
    })
  )
  .subscribe((result) => {
    if (result.isValid) {
      emailStatus.textContent = '✓ ' + result.message;
      emailStatus.className = 'valid';
    } else {
      emailStatus.textContent = '✗ ' + result.message;
      emailStatus.className = 'invalid';
    }
  });
```

## Transformation et agrégation de tableaux d'objets

```ts
import { from } from 'rxjs';
import { map, toArray } from 'rxjs';

// Données de ventes
const sales = [
  { product: 'PC portable', price: 120000, quantity: 3 },
  { product: 'Tablette', price: 45000, quantity: 7 },
  { product: 'Smartphone', price: 85000, quantity: 4 },
  { product: 'Souris', price: 3500, quantity: 12 },
  { product: 'Clavier', price: 6500, quantity: 8 },
];

// Transformation et agrégation des données
from(sales)
  .pipe(
    // Calcule le montant total pour chaque produit
    map((item) => ({
      product: item.product,
      price: item.price,
      quantity: item.quantity,
      total: item.price * item.quantity,
    })),
    // Ajoute le prix TTC
    map((item) => ({
      ...item,
      totalWithTax: Math.round(item.total * 1.1),
    })),
    // Reconvertit en tableau
    toArray(),
    // Calcule le montant total
    map((items) => {
      const grandTotal = items.reduce((sum, item) => sum + item.total, 0);
      const grandTotalWithTax = items.reduce(
        (sum, item) => sum + item.totalWithTax,
        0
      );
      return {
        items,
        grandTotal,
        grandTotalWithTax,
      };
    })
  )
  .subscribe((result) => {
    console.log('Détails des produits :', result.items);
    console.log('Montant total (HT) :', result.grandTotal);
    console.log('Montant total (TTC) :', result.grandTotalWithTax);
  });
```

## Normalisation des données JSON

```ts
import { ajax } from 'rxjs/ajax';
import { map } from 'rxjs';

const resultBox = document.createElement('div');
resultBox.id = 'normalized-results';
document.body.appendChild(resultBox);

ajax
  .getJSON<any[]>('https://jsonplaceholder.typicode.com/users')
  .pipe(
    map((users) => {
      // Convertit en objet avec ID comme clé
      const normalizedUsers: Record<number, any> = {};
      const userIds: number[] = [];

      users.forEach((user) => {
        normalizedUsers[user.id] = {
          ...user,
          // Aplatit les objets imbriqués
          companyName: user.company.name,
          city: user.address.city,
          street: user.address.street,
          // Supprime les imbrications inutiles
          company: undefined,
          address: undefined,
        };
        userIds.push(user.id);
      });

      return {
        entities: normalizedUsers,
        ids: userIds,
      };
    })
  )
  .subscribe((result) => {
    const title = document.createElement('h3');
    title.textContent = 'Données utilisateurs normalisées';
    resultBox.appendChild(title);

    result.ids.forEach((id) => {
      const user = result.entities[id];
      const div = document.createElement('div');
      div.innerHTML = `
      <strong>${user.name}</strong><br>
      Nom d'utilisateur : @${user.username}<br>
      Email : ${user.email}<br>
      Entreprise : ${user.companyName}<br>
      Adresse : ${user.city}, ${user.street}<br><br>
    `;
      resultBox.appendChild(div);
    });

    // Accès rapide à un utilisateur par ID spécifique
    console.log('Utilisateur ID 3 :', result.entities[3]);
  });
```

## Combinaison de plusieurs transformations

Dans les applications réelles, il est courant d'utiliser une combinaison de plusieurs opérateurs de transformation.

```ts
import { fromEvent, timer } from 'rxjs';
import {
  switchMap,
  map,
  tap,
  debounceTime,
  takeUntil,
  distinctUntilChanged,
} from 'rxjs';
import { ajax } from 'rxjs/ajax';

type User = {
  id: number;
  name: string;
  username: string;
  email: string;
  company: {
    name: string;
  };
};

// Entrée de recherche
const searchInput = document.createElement('input');
const resultsContainer = document.createElement('p');
const loadingIndicator = document.createElement('p');

document.body.append(searchInput);
document.body.append(resultsContainer);
document.body.append(loadingIndicator);

// Traitement de recherche
fromEvent(searchInput, 'input')
  .pipe(
    // Récupère la valeur d'entrée
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // Attend 300ms
    debounceTime(300),
    // Ignore si même valeur
    distinctUntilChanged(),
    // Affiche le chargement
    tap(() => {
      loadingIndicator.style.display = 'block';
      resultsContainer.innerHTML = '';
    }),
    // Requête API (annule la requête précédente)
    switchMap((term) => {
      // Entrée vide = pas de résultats
      if (term === '') {
        return [];
      }

      // Traitement de timeout (5 secondes)
      const timeout$ = timer(5000).pipe(
        tap(() => console.warn('Timeout de la réponse API')),
        map(() => [{ error: 'Timeout' }])
      );

      // Appel API
      const response$ = ajax
        .getJSON(
          `https://jsonplaceholder.typicode.com/users?username_like=${term}`
        )
        .pipe(
          // Traite les résultats
          map((users) =>
            (users as User[]).map((user) => ({
              id: user.id,
              name: user.name,
              username: user.username,
              email: user.email,
              company: user.company.name,
            }))
          ),
          // Complete avant le timeout
          takeUntil(timeout$)
        );

      return response$;
    }),
    // Fin du chargement
    tap(() => {
      loadingIndicator.style.display = 'none';
    })
  )
  .subscribe((result) => {
    loadingIndicator.style.display = 'none';

    if (Array.isArray(result)) {
      if (result.length === 0) {
        resultsContainer.innerHTML =
          '<div class="no-results">Aucun utilisateur trouvé</div>';
      } else {
        resultsContainer.innerHTML = result
          .map(
            (user) => `
          <div class="user-card">
            <h3>${user.name}</h3>
            <p>@${user.username}</p>
            <p>${user.email}</p>
            <p>Entreprise : ${user.company}</p>
          </div>
        `
          )
          .join('');
      }
    } else {
      resultsContainer.innerHTML = `<div class="error">⚠️ ${result}</div>`;
    }
  });
```

## 🧠 Résumé

- Pour les transformations simples, utilisez `map`
- `mergeMap`, `switchMap`, `concatMap`, `exhaustMap` pour gérer les opérations asynchrones
- `bufferTime` et `bufferCount` pour le traitement par lots
- `pluck` pour l'extraction de propriétés
- Dans les applications réelles, **ils sont généralement combinés**

Une fois que vous avez maîtrisé les opérateurs de transformation, vous pouvez gérer des flux de données asynchrones complexes de manière intuitive et déclarative !
