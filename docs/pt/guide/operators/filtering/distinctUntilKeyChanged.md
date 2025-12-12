---
description: O operador distinctUntilKeyChanged foca em uma propriedade específica dentro de um stream de objetos e emite apenas quando aquele valor difere do anterior. Ele ignora eficientemente dados duplicados consecutivos e é útil para detectar mudanças de estado e otimizar atualizações de lista.
titleTemplate: ':title'
---

# distinctUntilKeyChanged - Detectar Mudanças Apenas em Propriedade Específica

O operador `distinctUntilKeyChanged` foca em uma chave específica (propriedade) de um objeto e emite apenas quando aquele valor difere do anterior.
É útil para ignorar eficientemente duplicatas consecutivas.


## 🔰 Sintaxe Básica e Uso

```ts
import { from } from 'rxjs';
import { distinctUntilKeyChanged } from 'rxjs';

const users = [
  { id: 1, name: 'Tanaka' },
  { id: 2, name: 'Tanaka' }, // Mesmo nome, ignorar
  { id: 3, name: 'Sato' },
  { id: 4, name: 'Suzuki' },
  { id: 5, name: 'Suzuki' }, // Mesmo nome, ignorar
  { id: 6, name: 'Tanaka' }
];

from(users).pipe(
  distinctUntilKeyChanged('name')
).subscribe(console.log);

// Saída:
// { id: 1, name: 'Tanaka' }
// { id: 3, name: 'Sato' }
// { id: 4, name: 'Suzuki' }
// { id: 6, name: 'Tanaka' }
```

- Emite apenas quando o valor da propriedade especificada `name` muda.
- Outras propriedades (por exemplo, `id`) não são comparadas.

[🌐 Documentação Oficial RxJS - `distinctUntilKeyChanged`](https://rxjs.dev/api/operators/distinctUntilKeyChanged)


## 💡 Padrões de Uso Típicos

- Atualizar exibição de lista apenas quando uma propriedade específica muda
- Detectar apenas mudanças em atributos específicos em streams de eventos
- Controlar remoção de duplicatas com base em chave


## 🧠 Exemplo de Código Prático (com UI)

Digite um nome na caixa de texto e pressione Enter para registrá-lo.
**Se o mesmo nome for digitado consecutivamente, ele é ignorado**, e é adicionado à lista apenas quando um nome diferente é digitado.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, scan, distinctUntilKeyChanged } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
document.body.appendChild(output);

const title = document.createElement('h3');
title.textContent = 'Exemplo Prático de distinctUntilKeyChanged';
output.appendChild(title);

// Formulário de entrada
const input = document.createElement('input');
input.placeholder = 'Digite um nome e pressione Enter';
document.body.appendChild(input);

// Stream de evento de entrada
fromEvent<KeyboardEvent>(input, 'keydown').pipe(
  filter((e) => e.key === 'Enter'),
  map(() => input.value.trim()),
  filter((name) => name.length > 0),
  scan((_, name, index) => ({ id: index + 1, name }), { id: 0, name: '' }),
  distinctUntilKeyChanged('name')
).subscribe((user) => {
  const item = document.createElement('div');
  item.textContent = `Entrada do usuário: ID=${user.id}, Nome=${user.name}`;
  output.appendChild(item);
});
```

- Se o mesmo nome for digitado consecutivamente, ele é ignorado.
- É exibido apenas quando um novo nome é digitado.
